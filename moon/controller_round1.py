"""
  Space Robotics Challenge 2
"""
from datetime import timedelta

from osgar.bus import BusShutdownException
from osgar.lib import quaternion
from osgar.lib.virtual_bumper import VirtualBumper

from moon.controller import (SpaceRoboticsChallenge, ChangeDriverException, VirtualBumperException,
                             LidarCollisionException, LidarCollisionMonitor)


class SpaceRoboticsChallengeRound1(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def on_object_reached(self, data):
        object_type = data
        x,y,z = self.xyz
        print(self.sim_time, "app: Object %s reached" % object_type)


        def process_volatile_response(response):
            print(self.sim_time, "app: Volatile report response: %s" % response)
            if response == 'ok':
                pass
            else:
                # do nothing, ie keep going around and try to match the view
                pass

        self.send_request('artf %s %f %f 0.0' % (object_type, x, y), process_volatile_response)

    def run(self):
        def register_origin(message):
            print ("controller round 1: origin received: %s" % message)
        self.send_request('request_origin', register_origin)

        try:
            self.wait_for_init()
            last_walk_start = 0.0
            start_time = self.sim_time
            while self.sim_time - start_time < timedelta(minutes=40):
                additional_turn = 0
                last_walk_start = self.sim_time
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=4), 0.1)
                    with LidarCollisionMonitor(self):
                        if self.current_driver is None and not self.brakes_on:
                            self.go_straight(50.0, timeout=timedelta(minutes=2))
                        else:
                            self.wait(timedelta(minutes=2)) # allow for self driving, then timeout
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
                    self.turn(math.radians(deg_angle), timeout=timedelta(seconds=30))
                except ChangeDriverException as e:
                    continue

                except VirtualBumperException:
                    self.inException = True
                    print(self.sim_time, "Turn Virtual Bumper!")
                    self.virtual_bumper = None
                    if self.current_driver is not None:
                        # probably didn't throw in previous turn but during self driving
                        self.go_straight(-2.0, timeout=timedelta(seconds=10))
                        self.try_step_around()
                    self.turn(math.radians(-deg_angle), timeout=timedelta(seconds=30))
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
