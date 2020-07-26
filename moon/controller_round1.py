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

from moon.controller import (SpaceRoboticsChallenge, VirtualBumperException,
                             LidarCollisionException, LidarCollisionMonitor)

class SpaceRoboticsChallengeRound1(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.use_gimbal = False

        self.volatile_queue = [] # each element is an array [index, name x, y]
        self.volatile_last_submit_time = None

    def on_vslam_pose(self, data):
        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return
        super().on_vslam_pose(data)
        if not math.isnan(data[0][0]) and self.tf['vslam']['trans_matrix'] is None:
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

    def on_object_reached(self, data):
        object_type, object_index = data

        if (
                self.tf['vslam']['trans_matrix'] is not None and
                self.tf['vslam']['timestamp'] is not None and
                self.sim_time - self.tf['vslam']['timestamp'] < timedelta(milliseconds=300)
        ):
            x,y,z = self.xyz
            print(self.sim_time, "app: Object %s[%d] reached at [%.1f,%.1f], queueing up 9 around" % (object_type, object_index, x, y))
            for i in range(-2,2,2):
                for j in range(-2,2,2):
                    self.volatile_queue.append([object_index, object_type, x + i, y + j])

    def run(self):

        # chord length=2*sqrt(h * (2* radius - h)) where h is the distance from the circle boundary
        # https://mathworld.wolfram.com/CircularSegment.html
        # TODO: queue unsuccessful hits and then report in 30 seconds intervals 8 additional volatiles 2m in each direction


        # generate sweep trajectory for searching a square
        sweep_steps = []
        current_sweep_step = 0
        for i in range(15):
            sweep_steps.append([-20, -20+2*i])
            sweep_steps.append([20, -20+2*i])
            sweep_steps.append([20, -20+2*(i+1)])
            sweep_steps.append([-20, -20+2*(i+1)])

        try:
            self.wait_for_init()
            start_time = self.sim_time
            self.set_light_intensity("0.1")

            # TODO add 'try' or wait otherwise
            # TODO: drive around to build a basic map around starting point first
            # call request_origin after some basic map was built and stabilized
            self.go_straight(1, timeout=timedelta(minutes=2))

            while self.sim_time - start_time < timedelta(minutes=40):
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=4), 0.1)
                    with LidarCollisionMonitor(self):
                        self.go_to_location(sweep_steps[current_sweep_step][0], sweep_steps[current_sweep_step][1])
                        # if completed or timed out, move on; if exception, try again
                        current_sweep_step += 1
                        continue

                except (VirtualBumperException, LidarCollisionException) as e:
                    self.inException = True
                    print(self.sim_time, repr(e))
                    self.virtual_bumper = None
                    self.go_straight(-2.0, timeout=timedelta(seconds=10))
                    self.try_step_around()
                    self.inException = False

        except BusShutdownException:
            pass


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Space Robotics Challenge 2')
    args = parser.parse_args()

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
