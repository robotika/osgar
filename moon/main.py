"""
  Space Robotics Challenge 2
"""

import math
from random import Random
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib import quaternion
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.lib.virtual_bumper import VirtualBumper


PRINT_STATUS_PERIOD = timedelta(seconds=10)


class VirtualBumperException(Exception):
    pass


def distance(pose1, pose2):
    return math.hypot(pose1[0] - pose2[0], pose1[1] - pose2[1])


def min_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 10000 for x in laser_data]
        return min(laser_data)/1000.0
    return 0


class SpaceRoboticsChallenge(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_speed", "artf_cmd", "pose2d", "pose3d", "request_origin")
        self.last_position = None
        self.max_speed = 1.0  # oficial max speed is 1.5m/s
        self.max_angular_speed = math.radians(60)

        self.origin = None  # unknown initial position
        self.origin_quat = quaternion.identity()
        self.yaw_offset = 0.0  # on Moon is the rover placed randomly, but orientation corresponds to IMU
        self.yaw, self.pitch, self.roll = 0, 0, 0
        self.xyz = (0, 0, 0)  # 3D position for mapping artifacts
        self.xyz_quat = [0, 0, 0]  # pose3d using Quaternions
        self.offset = (0, 0, 0)
        self.score = None
        self.orientation = None  # channel data

        self.last_artf = None
        
        self.inException = False
        
        self.last_volatile_distance = None
        self.last_vol_index = None

        self.last_status_timestamp = None
        
        self.virtual_bumper = None
        self.rand = Random(0)
        self.verbose = False

    def send_speed_cmd(self, speed, angular_speed):
        if self.virtual_bumper is not None:
            self.virtual_bumper.update_desired_speed(speed, angular_speed)
        self.bus.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def on_pose2d(self):
        x, y, heading = self.pose2d
        pose = (x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
        if self.last_position is not None:
            dist = math.hypot(pose[0] - self.last_position[0], pose[1] - self.last_position[1])
            direction = ((pose[0] - self.last_position[0]) * math.cos(self.last_position[2]) +
                         (pose[1] - self.last_position[1]) * math.sin(self.last_position[2]))
            if direction < 0:
                dist = -dist
        else:
            dist = 0.0
        self.last_position = pose
        x, y, z = self.xyz
        x += math.cos(self.pitch) * math.cos(self.yaw) * dist
        y += math.cos(self.pitch) * math.sin(self.yaw) * dist
        z += math.sin(self.pitch) * dist
        x0, y0, z0 = self.offset
        self.last_send_time = self.bus.publish('pose2d', [round((x + x0) * 1000), round((y + y0) * 1000),
                                    round(math.degrees(self.yaw) * 100)])
        self.xyz = x, y, z

        # pose3d
        dist3d = quaternion.rotate_vector([dist, 0, 0], self.orientation)
        self.xyz_quat = [a + b for a, b in zip(self.xyz_quat, dist3d)]
        self.bus.publish('pose3d', [self.xyz_quat, self.orientation])

        if self.virtual_bumper is not None:
            self.virtual_bumper.update_pose(self.time, pose)
            if not self.inException and self.virtual_bumper.collision():
                self.inException = True
                raise VirtualBumperException()

    def on_artf(self):
        data = self.artf
        if self.verbose:
            print(data)
        # called by incoming volatile sensor report (among other sources)
        # 0 vol_type, 1 distance_to, 2 vol_index
        artifact_type = data[0]  # meters ... TODO distinguish CubeSat, volatiles, ProcessingPlant
        if artifact_type == "CubeSat" or artifact_type == "ProcessingPlant":
            return

        if self.last_artf is None:
            self.bus.publish('request_origin', True)
        self.last_artf = artifact_type
        
        distance_to = data[1]
        vol_index = data[2]

        if self.last_volatile_distance is None:
            self.last_volatile_distance = distance_to
        elif self.last_volatile_distance > distance_to:
            if self.verbose:
                print(self.time, "Volatile detection %d, getting closer: %f" % (vol_index, distance_to))
            self.last_volatile_distance = distance_to
        elif self.last_vol_index is None or vol_index != self.last_vol_index:
            self.last_vol_index = vol_index
            self.last_volatile_distance = None
            # TODO: this must be adjusted to report the position of the sensor, not the robot (which NASA will update their code for at some point)
            # the best known distance was in reference to mutual position of the sensor and the volatile
            ax, ay, az = self.xyz
            ox, oy, oz = self.offset
            print(self.time, "Volatile %d detection at %.3fm, starting to go further, reporting %f %f" % 
                    (vol_index, distance_to, ax + ox, ay + oy))

            # TODO (maybe): if not accepted, try again?
            s = '%s %.2f %.2f %.2f\n' % (artifact_type, ax + ox, ay + oy, 0.0)
            self.publish('artf_cmd', bytes('artf ' + s, encoding='ascii'))
        else:
            self.last_volatile_distance = None
            if self.verbose:
                print(self.time, "Previously visited volatile %d, not reporting" % vol_index)

    def on_origin(self):
        data = self.origin[:]  # the same name is used for message as internal data
        self.origin = data[1:4]
        x, y, z = self.xyz
        ox, oy, oz = self.origin
        self.offset = (ox - x, oy - y, oz - z)

        qx, qy, qz, qw = data[4:]
        self.origin_quat = qx, qy, qz, qw  # quaternion
        print(self.time, "Origin received, internal position updated")

    def on_rot(self):
        temp_yaw, self.pitch, self.roll = [normalizeAnglePIPI(math.radians(x/100)) for x in self.rot]
        if self.yaw_offset is None:
            self.yaw_offset = -temp_yaw
        self.yaw = temp_yaw + self.yaw_offset
        if not self.inException and self.pitch > 0.5:
            self.inException = True
            raise VirtualBumperException()

    def update(self):
        if self.time is not None:
            if self.last_status_timestamp is None:
                self.last_status_timestamp = self.time
            elif self.time - self.last_status_timestamp > PRINT_STATUS_PERIOD:
                self.last_status_timestamp = self.time
                x, y, z = self.xyz
                ox, oy, oz = self.offset
                print(self.time, "Loc: %.2f %.2f %.2f; Score: " % (x + ox, y + oy, z + oz), self.score,
                      "%.2f %.2f %.2f" % tuple(self.xyz_quat))

        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler()
        return channel

    def go_straight(self, how_far, timeout=None):
        print(self.time, "go_straight %.1f (speed: %.1f)" % (how_far, self.max_speed), self.last_position)
        start_pose = self.last_position
        if how_far >= 0:
            self.send_speed_cmd(self.max_speed, 0.0)
        else:
            self.send_speed_cmd(-self.max_speed, 0.0)
        start_time = self.time
        while distance(start_pose, self.last_position) < abs(how_far):
            self.update()
            if timeout is not None and self.time - start_time > timeout:
                print("go_straight - timeout at %.1fm" % distance(start_pose, self.last_position))
                break
        self.send_speed_cmd(0.0, 0.0)

    def turn(self, angle, with_stop=True, speed=0.0, timeout=None):
        print(self.time, "turn %.1f" % math.degrees(angle))
        if angle >= 0:
            self.send_speed_cmd(speed, self.max_angular_speed)
        else:
            self.send_speed_cmd(speed, -self.max_angular_speed)
        start_time = self.time
        # problem with accumulated angle

        sum_angle = 0.0
        prev_angle = self.yaw
        while abs(sum_angle) < abs(angle):
            self.update()
            sum_angle += normalizeAnglePIPI(self.yaw - prev_angle)
            prev_angle = self.yaw
            if timeout is not None and self.time - start_time > timeout:
                print(self.time, "turn - timeout at %.1fdeg" % math.degrees(sum_angle))
                break
        if with_stop:
            self.send_speed_cmd(0.0, 0.0)
            start_time = self.time
            while self.time - start_time < timedelta(seconds=2):
                self.update()
            print(self.time, 'stop at', self.time - start_time)

    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def run(self):
        try:
            print('Wait for definition of last_position, yaw and orientation')
            while self.last_position is None or self.yaw is None or self.orientation is None:
                self.update()  # define self.time
            print('done at', self.time)

            start_time = self.time
            while self.time - start_time < timedelta(minutes=40):
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=2), 0.1)
                    self.go_straight(100.0, timeout=timedelta(minutes=2))
                except VirtualBumperException:
                    print(self.time, "Virtual Bumper!")
                    self.virtual_bumper = None
                    self.go_straight(-1.0, timeout=timedelta(seconds=10))
                    self.inException = False

                deg_angle = self.rand.randrange(90, 180)
                deg_sign = self.rand.randint(0,1)
                if deg_sign:
                    deg_angle = -deg_angle
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=2), 0.1)
                    self.turn(math.radians(deg_angle), timeout=timedelta(seconds=30))
                except VirtualBumperException:
                    print(self.time, "Turn Virtual Bumper!")
                    self.virtual_bumper = None
                    self.turn(math.radians(-deg_angle), timeout=timedelta(seconds=30))
                    self.inException = False

            self.wait(timedelta(seconds=10))
        except BusShutdownException:
            pass


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Space Robotics Challenge 2')
    args = parser.parse_args()

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
