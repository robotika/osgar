"""
Robotour competition
https://robotika.cz/competitions/robotour/2020/cs
"""
import math
import numpy as np
from datetime import timedelta
import time

from osgar.node import Node


def get_direction(x_diff, y_diff):
    if x_diff == 0:
        return math.copysign(math.pi/2, y_diff)
    phi = math.atan(y_diff / x_diff)
    if x_diff < 0:
        return math.copysign(math.pi, y_diff) + phi
    return phi


def downsample_scan(scan):
    scan = np.array(scan)
    scan[scan < 10] = 9999
    scan = np.median(np.reshape(scan[:810], (3, 270)), axis=0)
    return scan


class Robotour(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.speed = config['max_speed']
        self.pose = [0, 0, 0]
        self.verbose = False
        self.destination = [3, 0]
        self.new_scan = None
        self.dangerous_dist = 0.35
        self.min_safe_dist = 2


    def update(self):
        channel = super().update()  # define self.time
        if self.verbose:
            print(self.time, 'Robotour', channel)
        if channel == 'pose2d':
            x, y, heading = self.pose2d
            self.pose = (x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
        if channel == "scan":
            assert len(self.scan) == 811, len(self.scan)
            self.new_scan = downsample_scan(self.scan)


    def send_speed_cmd(self, speed, angular_speed):
        return self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])


    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def go_safely(self, desired_direction):
        angular_speed = desired_direction
        size = len(self.new_scan)
        dist = np.min(self.new_scan[size // 3:2 * size // 3]) / 1000
        if dist > self.min_safe_dist:
            self.send_speed_cmd(self.speed, angular_speed)
        else:
            if dist < self.dangerous_dist:
                self.send_speed_cmd(0, 0)
                print(self.time, "Dangerous dist!")
                return False

            speed = self.speed * (dist - self.dangerous_dist) / (self.min_safe_dist - self.dangerous_dist)
            speed = max(0.25, speed)
            self.send_speed_cmd(speed, angular_speed)

        return True


    def navigate(self, direction):
        scan = self.new_scan / 1000
        # use max feasible direction
        direction = math.copysign(min(abs(direction), math.pi/2), direction)
        direction_id = round(math.degrees(direction)) + 135

        binarized = scan>1.2
        #print(binarized)
        possible_directions = np.convolve(binarized, np.ones(80) / 80, mode="valid")
        possible_directions_idx = np.where(possible_directions > 0.999)[0] + 40  # It should be ==1, strange behavior on the apu
        #print(possible_directions_idx)
        if len(possible_directions_idx) == 0:
            return None

        idx = np.argmin(abs(possible_directions_idx - direction_id))
        #print(idx)
        new_direction = possible_directions_idx[idx]
        safe_direction = math.radians(new_direction - 135)

        return safe_direction


    def run(self):
        self.update()  # define self.time
        print(self.time, "Go!")
        start_time = self.time
        while self.time - start_time < timedelta(seconds=20):
            if self.new_scan is not None:
                x, y, heading = self.pose
                y_diff = self.destination[1] - y
                x_diff = self.destination[0] - x
                destination_dist = math.hypot(x_diff, y_diff )
                if destination_dist < 1:
                    print("Destination reached")
                    break

                direction = get_direction(x_diff, y_diff) - heading
                #t0 = time.time()
                safe_direction = self.navigate(direction)
                #print(time.time()-t0)

                if safe_direction:
                    #print(self.time, direction, safe_direction)
                    self.go_safely(safe_direction)

                else:
                    print(self.time, "NO SAVE DIRECTION!!!")
                    self.send_speed_cmd(0, 0)
            self.new_scan = None
            self.update()

        self.send_speed_cmd(0.0, 0.0)
        self.wait(timedelta(seconds=2))


if __name__ == "__main__":
    from osgar.launcher import launch

    launch(app=Robotour, description='Robotour competition', prefix='Robotour-')
