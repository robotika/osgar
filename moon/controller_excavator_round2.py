"""
  Space Robotics Challenge 2
"""
from moon.controller import SpaceRoboticsChallenge


class SpaceRoboticsChallengeExcavatorRound2(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def run(self):
        self.wait_for_init()

        def process_volatiles(vol_list):
            print (self.time, "main-excavator-round2: Volatiles: %s" % vol_list)
        self.send_request('get_volatile_locations', process_volatiles)

        super().run()

# vim: expandtab sw=4 ts=4
