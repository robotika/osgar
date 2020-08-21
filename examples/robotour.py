"""
Robotour competition
https://robotika.cz/competitions/robotour/2020/cs
"""
import math
from datetime import timedelta

from osgar.node import Node


def get_direction(x_diff, y_diff):
    if x_diff == 0:
        return math.copysign(math.pi/2, y_diff)
    phi = math.atan(y_diff / x_diff)
    if x_diff < 0:
        return math.copysign(math.pi, y_diff) + phi
    return phi


class Robotour(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.speed = config['max_speed']
        self.pose = [0, 0, 0]
        self.verbose = False
        self.destination = [1, 2]


    def update(self):
        channel = super().update()  # define self.time
        if self.verbose:
            print(self.time, 'Robotour', channel)
        if channel == 'pose2d':
            x, y, heading = self.pose2d
            self.pose = (x / 1000.0, y / 1000.0, math.radians(heading / 100.0))


    def send_speed_cmd(self, speed, angular_speed):
        return self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])


    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()


    def run(self):
        self.update()  # define self.time
        print(self.time, "Go!")
        start_time = self.time
        while self.time - start_time < timedelta(seconds=20):
            x, y, heading = self.pose
            y_diff = self.destination[1] - y
            x_diff = self.destination[0] - x
            destination_dist = math.hypot(x_diff, y_diff )
            if destination_dist < 1:
                print("Destination reached")
                break

            direction = get_direction(x_diff, y_diff)

            angular_speed = direction - heading
            self.send_speed_cmd(self.speed, angular_speed)
            self.update()

        self.send_speed_cmd(0.0, 0.0)
        self.wait(timedelta(seconds=2))


if __name__ == "__main__":
    from osgar.launcher import launch

    launch(app=Robotour, description='Robotour competition', prefix='Robotour-')
