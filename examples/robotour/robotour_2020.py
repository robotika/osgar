"""
Robotour competition
https://robotika.cz/competitions/robotour/2020/cs
"""
import math
import numpy as np
import cv2
from pyzbar.pyzbar import decode
from datetime import timedelta

from osgar.node import Node

R = 12720000/2 #  Earth radius [m]


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
    scan = np.median(np.reshape(scan[:810], (270, 3)), axis=1)
    return scan


def parse_gps(data):
    #gps = eval(data[4:]) #  in the beginning is "geo:"
    gps = eval(data)
    if len(gps) == 2:
        return gps


class Robotour(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.speed = config['max_speed']
        self.pose = [0, 0, 0]
        self.verbose = False
        self.destination = None
        self.new_scan = None
        self.last_image = None #  numpy image
        self.dangerous_dist = 0.35
        self.min_safe_dist = 1.5
        self.gps_start = (50.1286131, 14.3748433)
        self.x_scale = math.cos(math.radians(self.gps_start[0]))


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
        if channel =="image":
            self.last_image = cv2.imdecode(np.frombuffer(self.image, np.uint8), 1)
            #print(type(self.last_image), self.last_image.shape)


    def send_speed_cmd(self, speed, angular_speed):
        return self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])


    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()


    def latlon_diff2xy(self, dest_gps):
        x = R * math.radians(dest_gps[1] - self.gps_start[1]) * self.x_scale
        y = R * math.radians(dest_gps[0] - self.gps_start[0])
        return x, y


    def get_qrcode(self):
        qr_codes = decode(self.last_image)
        if len(qr_codes) > 0:
            for qr in qr_codes:
                print(qr.data)
                dest_gps = parse_gps(qr.data)
                dest = self.latlon_diff2xy(dest_gps)

                return dest


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
        possible_directions = np.convolve(binarized, np.ones(80) / 80, mode="valid")
        possible_directions_idx = np.where(possible_directions > 0.999)[0] + 40  # It should be ==1, strange behavior on the apu
        if len(possible_directions_idx) == 0:
            return None

        idx = np.argmin(abs(possible_directions_idx - direction_id))
        #print(idx)
        new_direction = possible_directions_idx[idx]
        safe_direction = math.radians(new_direction - 135)

        return safe_direction

    def ask4beer(self):
        pass

    def wait4qr_code(self):
        print("waiting for QR code..")
        while True:
            if self.last_image is not None:
                destination = self.get_qrcode()
                if destination is not None:
                    print(destination)
                    break
                self.last_image = None
            self.update()
        return destination

    def compute_path(self, destination):
        return [[3,0], [3,-3], [0,-3]]

    def go2way_point(self, way_point):
        while True:
            if self.new_scan is not None:
                x, y, heading = self.pose
                y_diff = way_point[1] - y
                x_diff = way_point[0] - x
                destination_dist = math.hypot(x_diff, y_diff )
                if destination_dist < 1:
                    print("Destination reached")
                    break

                direction = get_direction(x_diff, y_diff) - heading
                #t0 = time.time()
                safe_direction = self.navigate(direction)
                #print(time.time()-t0)

                if safe_direction is not None:
                    #print(self.time, direction, safe_direction)
                    self.go_safely(safe_direction)

                else:
                    print(self.time, "NO SAVE DIRECTION!!!")
                    self.send_speed_cmd(0, 0)
                self.new_scan = None
            self.update()

    def run(self):
        self.update()  # define self.time
        destination = self.wait4qr_code()
        path = self.compute_path(destination)
        print(self.time, "Go!")
        for way_point in path:
            self.go2way_point(way_point)
        self.send_speed_cmd(0.0, 0.0)
        self.ask4beer()

        destination = self.wait4qr_code()
        path = self.compute_path(destination)
        print(self.time, "Go!")
        for way_point in path:
            self.go2way_point(way_point)
        self.send_speed_cmd(0.0, 0.0)
        self.ask4beer()  # unload beer

        path = self.compute_path((0,0))
        print(self.time, "Go!")
        for way_point in path:
            self.go2way_point(way_point)
        self.send_speed_cmd(0.0, 0.0)

        self.wait(timedelta(seconds=2))


if __name__ == "__main__":
    from osgar.launcher import launch

    launch(app=Robotour, description='Robotour competition', prefix='Robotour-')
