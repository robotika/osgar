"""
  Space Robotics Challenge 2
"""
from moon.controller import SpaceRoboticsChallenge


class SpaceRoboticsChallengeHaulerRound2(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def run(self):
        self.wait_for_init()
        super().run()

        
# vim: expandtab sw=4 ts=4
