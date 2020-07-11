"""
  Space Robotics Challenge 2
"""
from osgar.lib import quaternion
from moon.controller import SpaceRoboticsChallenge


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

        super().run()  # TODO refactor and move the code here (shared is not used/needed)


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Space Robotics Challenge 2')
    args = parser.parse_args()

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
