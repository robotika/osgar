"""
  Space Robotics Challenge 2
"""
import zmq

from moon.controller import SpaceRoboticsChallenge

class SpaceRoboticsChallengeExcavatorRound2(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def run(self):

        vol_list = self.send_request('get_volatile_locations').decode('ascii')
        print (self.time, "main-excavator-round2: Volatiles: %s" % vol_list)

        super().run()

# vim: expandtab sw=4 ts=4
