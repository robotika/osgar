"""
  Space Robotics Challenge 2
"""
import math
from datetime import timedelta

from osgar.bus import BusShutdownException
from osgar.lib import quaternion
from osgar.lib.quaternion import euler_zyx

from osgar.lib.virtual_bumper import VirtualBumper

from moon.controller import (SpaceRoboticsChallenge, ChangeDriverException, VirtualBumperException,
                             LidarCollisionException, LidarCollisionMonitor)

class SpaceRoboticsChallengeRound1(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.use_gimbal = False

    def on_vslam_pose(self, data):
        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return
        super().on_vslam_pose(data)
        if not math.isnan(data[0][0]) and self.tf['vslam']['trans_matrix'] is None:
            # request origin and start tracking in correct coordinates as soon as first mapping lock occurs
            # TODO: another pose may arrive while this request is still being processed (not a big deal, just a ROS error message)
            self.send_request('request_origin', self.register_origin)


    def process_volatile_response(self, response):
        print(self.sim_time, "app: Volatile report response: %s" % response)
        if response == 'ok':
            pass
        else:
            # do nothing, ie keep going around and try to match the view
            pass

    def on_object_reached(self, data):
        object_type = data

        def register_and_report(message):
            self.register_origin(message)
            x,y,z = self.xyz
            print(self.sim_time, "app: Object %s reached" % object_type)
            self.send_request('artf %s %f %f 0.0' % (object_type, x, y), self.process_volatile_response)

        if (
                self.tf['vslam']['trans_matrix'] is not None and
                self.tf['vslam']['timestamp'] is not None and
                self.sim_time - self.tf['vslam']['timestamp'] < timedelta(milliseconds=300)
        ):
            x,y,z = self.xyz
            print(self.sim_time, "app: Object %s reached" % object_type)
            self.send_request('artf %s %f %f 0.0' % (object_type, x, y), self.process_volatile_response)

    def run(self):

        dist = [10, 20, 30, 50]
        dist_index = 0
        dist_counter = 0
        DIST_THRES = 20

        try:
            self.wait_for_init()
            self.set_light_intensity("0.1")
            last_walk_start = 0.0
            start_time = self.sim_time
            while self.sim_time - start_time < timedelta(minutes=40):
                additional_turn = 0
                last_walk_start = self.sim_time
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=4), 0.1)
                    with LidarCollisionMonitor(self):
                        if self.current_driver is None and not self.brakes_on:
                            self.go_straight(dist[dist_index], timeout=timedelta(minutes=2))
                            dist_counter += 1
                            if dist_counter > DIST_THRES and dist_index < len(dist) - 1:
                                dist_counter = 0
                                dist_index += 1
                        else:
                            self.wait(timedelta(minutes=2)) # allow for self driving, then timeout

                        # if further than 25m from the center, turn back
                        # will not use correct coordinates until true_pose is requested
                        # until then, it will use the robot spawning place as the center which is fine
                        if math.hypot(self.xyz[0], self.xyz[1]) > 25:
                            print(self.sim_time, "app: too far from center, turning towards center")
                            angle = math.atan2(self.xyz[1], self.xyz[0]) - self.yaw
                            self.turn(angle, timeout=timedelta(seconds=10))
                            continue
                    self.update()
                except ChangeDriverException as e:
                    continue

                except (VirtualBumperException, LidarCollisionException) as e:
                    self.inException = True
# TODO: crashes if an exception (e.g., excess pitch) occurs while handling an exception (e.g., virtual/lidar bump)
                    print(self.sim_time, repr(e))
                    last_walk_end = self.sim_time
                    self.virtual_bumper = None
                    self.go_straight(-2.0, timeout=timedelta(seconds=10))
                    if last_walk_end - last_walk_start > timedelta(seconds=20): # if we went more than 20 secs, try to continue a step to the left
                        self.try_step_around()
                    else:
                        self.bus.publish('driving_recovery', False)

                    self.inException = False

                    print ("Time elapsed since start of previous leg: %d sec" % (last_walk_end.total_seconds()-last_walk_start.total_seconds()))
                    if last_walk_end - last_walk_start > timedelta(seconds=20):
                        # if last step only ran short time before bumper, time for a large random turn
                        # if it ran long time, maybe worth trying going in the same direction
                        continue
                    additional_turn = 30

                    # next random walk direction should be between 30 and 150 degrees
                    # (no need to go straight back or keep going forward)
                    # if part of virtual bumper handling, add 30 degrees to avoid the obstacle more forcefully

                deg_angle = self.rand.randrange(30 + additional_turn, 150 + additional_turn)
                deg_sign = self.rand.randint(0,1)
                if deg_sign:
                    deg_angle = -deg_angle
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)
                    with LidarCollisionMonitor(self):
                        self.turn(math.radians(deg_angle), timeout=timedelta(seconds=30))
                except ChangeDriverException as e:
                    continue

                except (VirtualBumperException, LidarCollisionException):
                    self.inException = True
                    print(self.sim_time, "Turn Virtual Bumper!")
                    self.virtual_bumper = None
                    if self.current_driver is not None:
                        # probably didn't throw in previous turn but during self driving
                        self.go_straight(-2.0, timeout=timedelta(seconds=10))
                        self.try_step_around()
                    self.turn(math.radians(-deg_angle/2), timeout=timedelta(seconds=30))
                    self.inException = False
                    self.bus.publish('driving_recovery', False)
        except BusShutdownException:
            pass


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Space Robotics Challenge 2')
    args = parser.parse_args()

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
