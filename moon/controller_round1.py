"""
  Space Robotics Challenge 2
"""
import math
from datetime import timedelta
from functools import partial

from osgar.bus import BusShutdownException
from osgar.lib import quaternion
from osgar.lib.quaternion import euler_zyx

from osgar.lib.virtual_bumper import VirtualBumper

from moon.controller import (SpaceRoboticsChallenge, VirtualBumperException, ChangeDriverException,  VSLAMLostException, VSLAMFoundException,
                             LidarCollisionException, LidarCollisionMonitor)

SPEED_ON = 10 # only +/0/- matters

class SpaceRoboticsChallengeRound1(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.use_gimbal = False

        self.volatile_queue = [] # each element is an array [index, name x, y]
        self.volatile_last_submit_time = None
        self.processing_plant_found = False
        self.vslam_reset_at = None
        self.returning_to_base = False

    def on_vslam_pose(self, data):
        super().on_vslam_pose(data)

        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return

        if math.isnan(data[0][0]): # VSLAM not tracking
            if self.tf['vslam']['trans_matrix'] is not None and not self.inException and not self.returning_to_base: # it was tracking so it is lost, go back and re-acquire lock
                raise VSLAMLostException
            return

        if not math.isnan(data[0][0]) and not self.inException and self.returning_to_base:
            self.returning_to_base = False
            raise VSLAMFoundException

        if self.vslam_reset_at is not None and self.sim_time - self.vslam_reset_at > timedelta(seconds=3) and not math.isnan(data[0][0]) and self.tf['vslam']['trans_matrix'] is None:
            # request origin and start tracking in correct coordinates as soon as first mapping lock occurs
            # TODO: another pose may arrive while this request is still being processed (not a big deal, just a ROS error message)
            self.send_request('request_origin', self.register_origin)

    def process_volatile_response(self, response, vol_index):
        print(self.sim_time, "app: Volatile report response: %s" % response)
        if response == 'ok':
            # if ok, no need to attempt the remaining guesses
            while len(self.volatile_queue) > 0:
                if self.volatile_queue[0][0] == vol_index:
                    self.volatile_queue.pop(0)
                else:
                    break
        else:
            # do nothing, ie keep going around and try to match the view
            pass

    def on_sim_clock(self, t):
        super().on_sim_clock(t)
        if (
                self.sim_time is not None and
                len(self.volatile_queue) > 0 and
                (self.volatile_last_submit_time is None or self.sim_time - self.volatile_last_submit_time > timedelta(seconds=30))
        ):
            vol = self.volatile_queue.pop(0)
            self.volatile_last_submit_time = self.sim_time
            self.send_request('artf %s %f %f 0.0' % (vol[1], vol[2], vol[3]), partial(self.process_volatile_response, vol_index=vol[0]))

    def on_artf(self, data):
        artifact_type = data[0]
        if artifact_type == "homebase" and not self.processing_plant_found:
            self.processing_plant_found = True
            raise ChangeDriverException


    def on_object_reached(self, data):
        object_type, object_index = data

        if (
                self.tf['vslam']['trans_matrix'] is not None and
                self.tf['vslam']['timestamp'] is not None and
                self.sim_time - self.tf['vslam']['timestamp'] < timedelta(milliseconds=300)
        ):
            x,y,z = self.xyz
            print(self.sim_time, "app: Object %s[%d] reached at [%.1f,%.1f], queueing up 9 around" % (object_type, object_index, x, y))
            self.volatile_queue.append([object_index, object_type, x, y])
            for i in range(-2,4,2): # -2, 0, 2
                for j in range(-2,4,2):
                    if i == 0 and j == 0:
                        continue
                    self.volatile_queue.append([object_index, object_type, x + i, y + j])

    def run(self):

        # chord length=2*sqrt(h * (2* radius - h)) where h is the distance from the circle boundary
        # https://mathworld.wolfram.com/CircularSegment.html

        # generate sweep trajectory for searching a square
        sweep_steps = []
        current_sweep_step = 0
        x = [-26, 44]
        i = 1
        sweep_steps.append(["goto", [-11,-10, SPEED_ON]])
        sweep_steps.append(["turn", math.radians(90)])
        sweep_steps.append(["turn", math.radians(-180)])
        sweep_steps.append(["goto", [-27, -50, SPEED_ON]])
        for y in range (-50, 40, 4):
            sweep_steps.append(["goto", [x[i], y, SPEED_ON]])
            sweep_steps.append(["goto", [x[1-i], y, -SPEED_ON]])
            sweep_steps.append(["goside", [4, 0]])



        def set_homebase_found(response):
            self.vslam_reset_at = self.sim_time

        try:
            self.wait_for_init()
            start_time = self.sim_time
            self.set_light_intensity("0.1")

            try:
                self.turn(math.radians(360), timeout=timedelta(seconds=40))
            except ChangeDriverException as e:
                self.send_speed_cmd(0.0, 0.0)
            finally:
                self.send_request('vslam_reset', set_homebase_found)

            while not self.true_pose:
                self.update()
            self.wait(timedelta(seconds=2)) # wait to receive first poses after true_pose before acting upon the current location

            while current_sweep_step < len(sweep_steps) and self.sim_time - start_time < timedelta(minutes=60):
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=4), 0.1) # radius of "stuck" area
                    with LidarCollisionMonitor(self):
                        move, data = sweep_steps[current_sweep_step]
                        if move == "goto":
                            self.go_to_location(data[0], data[1], data[2])
                        elif move == "goside":
                            self.move_sideways(data[0], view_direction=data[1])
                        elif move == "turn":
                            self.turn(data, timeout=timedelta(seconds=10))
                        else:
                            assert(False, "Incorrect move queued")
                        # if completed or timed out, move on; if exception, try again
                        current_sweep_step += 1
                        continue
                except VSLAMLostException as e:
                    print("VSLAM lost")
                    self.returning_to_base = True
                    sweep_steps.insert(current_sweep_step, ["goto", [-11,-10, SPEED_ON]])
                    continue
                except VSLAMFoundException as e:
                    print("VSLAM found")
                    current_sweep_step += 1
                    continue
                except LidarCollisionException as e:
                    print(self.sim_time, "Lidar")
                    self.inException = True
                    self.go_straight(-2.0, timeout=timedelta(seconds=10))
                    self.inException = False
                    current_sweep_step += 1
                except VirtualBumperException as e:
                    move, data = sweep_steps[current_sweep_step] # what were we trying to do?
                    print(self.sim_time, "Bumper")
                    self.inException = True
                    if move == "goto":
                        self.go_straight(math.copysign(2.0, -data[2]), timeout=timedelta(seconds=10)) # go 2m in opposite direction
                    elif move == "goside":
                        self.move_sideways(math.copysign(1, -data[0]))
                    elif move == "turn":
                        self.turn(data/2, timeout=timedelta(seconds=10))
                    self.inException = False
                    current_sweep_step += 1

        except BusShutdownException:
            pass


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Space Robotics Challenge 2')
    args = parser.parse_args()

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
