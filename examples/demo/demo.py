"""
  Example of a simple application controling robot to move in figure 8
"""
import math
from datetime import timedelta

from osgar.node import Node


def distance(pose1, pose2):
    return math.hypot(pose1[0] - pose2[0], pose1[1] - pose2[1])


class MyApp(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.max_speed = config.get('max_speed', 0.1)
        self.max_angular_speed = math.radians(50)  # TODO config
        self.verbose = False
        self.last_position = (0, 0, 0)
        self.is_moving = False
        self.pose2d = None  # TODO should be defined by super().__init__()

    # TODO refactor to common "serializer"
    def send_speed_cmd(self, speed, angular_speed):
        return self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    # TODO refactor to common driver (used from sick2018 example)
    def go_straight(self, how_far):
        print(self.time, "go_straight %.1f" % how_far, self.last_position)
        start_pose = self.last_position
        if how_far >= 0:
            self.send_speed_cmd(self.max_speed, 0.0)
        else:
            self.send_speed_cmd(-self.max_speed, 0.0)
        while distance(start_pose, self.last_position) < abs(how_far):
            self.update()
        self.send_speed_cmd(0.0, 0.0)

    def turn(self, angle, with_stop=True):
        print(self.time, "turn %.1f" % math.degrees(angle))
        start_pose = self.last_position
        if angle >= 0:
            self.send_speed_cmd(0.0, self.max_angular_speed)
        else:
            self.send_speed_cmd(0.0, -self.max_angular_speed)
        while abs(start_pose[2] - self.last_position[2]) < abs(angle):
            self.update()
        if with_stop:
            self.send_speed_cmd(0.0, 0.0)
            start_time = self.time
            while self.time - start_time < timedelta(seconds=2):
                self.update()
                if not self.is_moving:
                    break
            print(self.time, 'stop at', self.time - start_time)

    def update(self):
        prev = self.pose2d
        channel = super().update()
        if channel == 'pose2d':
            x_mm, y_mm, heading_mdeg = self.pose2d
            self.last_position = (x_mm/1000.0, y_mm/1000.0,
                                  math.radians(heading_mdeg/100.0))
            self.is_moving = (prev != self.pose2d)

    def run0(self):
        print("MyApp example - figure 8!")
        step_size = 0.5  # meters
        deg90 = math.radians(90)

        for i in range(4):
            self.go_straight(step_size)
            self.turn(deg90)

        for i in range(4):
            self.go_straight(step_size)
            self.turn(-deg90)

        print("END")


    def run(self):
        print("SubT demo")
        deg90 = math.radians(90)

        self.go_straight(10.0)
        self.turn(-deg90)
        self.go_straight(10.0)
        self.turn(deg90)
        self.go_straight(10.0)

        print("END")


if __name__ == "__main__":
    from osgar.launcher import launch

    launch(app=MyApp, description='Navigate figure eight', prefix='myapp-')

# vim: expandtab sw=4 ts=4
