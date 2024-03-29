"""
  Driver for robot Robik from cortexpilot.com
"""

import ctypes
import struct
import math
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib import quaternion


# CPR = 9958 (ticks per revolution)
# wheel diameter D = 395 mm
# 1 Rev = 1241 mm
ENC_SCALE = 0.01  # Skiddy, Robik=1.241/9958
WHEEL_DISTANCE = 0.267  # Skiddy, Robik=0.88  # meters TODO confirm
RAMP_STEP = 0.1  # fractional number for speed in -1.0 .. 1.0

SOFT_SPEED_LIMIT = 1.0  # m/s ... software speed limit, abs() value
SOFT_ANGULAR_SPEED_LIMIT = math.radians(15)  # rad/s


def sint32_diff(a, b):
    return ctypes.c_int32(a - b).value


def speed2tank(desired_speed, desired_angular_speed):
    diff = desired_angular_speed
    # TODO axis computation
    # TODO ramps
    return desired_speed - diff, desired_speed + diff # left, right


class Cortexpilot(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('raw', 'encoders', 'emergency_stop', 'pose2d', 
                     'voltage', 'rotation', 'orientation', 'scan',
                     'dropped')

        self._buf = b''

        # commands
        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0
        self.cmd_flags = 0x40  # tank like commands 0x140 # Skiddy, turn 0x40 #0x41  # 0 = remote steering, PWM OFF, laser ON, TODO
        self.speeds = self.plain_speeds()

        # status
        self.emergency_stop = None  # uknown state
        self.pose = (0.0, 0.0, 0.0)  # x, y in meters, heading in radians (not corrected to 2PI)
        self.flags = None
        self.last_encoders = None
        self.yaw = None
        self.lidar_valid = False
        self.lidar_timestamp = 0
        self.uptime = None
        self.verbose = False
        self.last_cmd = (0, 0, None)  # motors + timestamp
        self.last_processed_cmd = None  # i.e. already accepted by cortexpilot
        self.watchdog_count = 0  # how many times was watchdog triggered?
        self.min_watchdog_dt = None

    def send_pose(self):
        x, y, heading = self.pose
        self.publish('pose2d', [round(x*1000), round(y*1000),
                                round(math.degrees(heading)*100)])

    def query_version(self):
        ret = bytes([0, 0, 3, 0x1, 0x01])
        checksum = sum(ret) & 0xFF
        return ret + bytes([(256-checksum) & 0xFF])

    def oscilate(self):
        while True:
            end = self.time + timedelta(seconds=1)
            while self.time < end:
                yield self.desired_speed, -self.desired_angular_speed
            end = self.time + timedelta(seconds=1)
            while self.time < end:
                yield -self.desired_speed, -self.desired_angular_speed

    def plain_speeds(self):
        while True:
            yield self.desired_speed, -self.desired_angular_speed

    def create_packet(self):
        if self.yaw is None:
            self.yaw = 0.0  # hack!

        desired_speed, desired_angular_speed = next(self.speeds)

        # limit desired speeds (before application of correction scale factors)
        # to protect robot and environment from SW bugs
        desired_speed = max(-SOFT_SPEED_LIMIT, min(SOFT_SPEED_LIMIT, desired_speed))
        desired_angular_speed = max(-SOFT_ANGULAR_SPEED_LIMIT, min(SOFT_ANGULAR_SPEED_LIMIT, desired_angular_speed))

        # this is left and right
        speed_frac, speed_dir = speed2tank(desired_speed, desired_angular_speed)

#        if speed_frac < 0:
#            speed_dir = -speed_dir  # Robik V5.1.1 handles backup backwards
        if self.emergency_stop:  #not self.lidar_valid:
            speed_frac = 0.0
            speed_dir = 0.0

        self.last_processed_cmd = self.last_cmd
        self.last_cmd = (speed_frac, speed_dir, self.time)

        #print(self.time, "{:.4f}, {:.4f} \t {:.4f} {:.4f}".format(speed_frac, speed_dir, self.desired_speed, self.desired_angular_speed))

        flags = self.cmd_flags
        #  flags |= (1<<8)  # agresive turning - actually for Skiddy this mean directional control
        if self.emergency_stop is not None:
            if self.emergency_stop:
                flags |= (1<<11)  # display red LEDs
            else:
                flags |= (1<<10)  # turn on green (9th bit)

        packet = struct.pack('<ffI', speed_frac, speed_dir, flags)
        assert len(packet) < 256, len(packet)  # just to use LSB only
        ret = bytes([0, 0, len(packet) + 2 + 1, 0x1, 0x0E]) + packet
        # addr=0x1, cmd=0xE, length is given by payload, addr, cmd and checksum
        checksum = sum(ret) & 0xFF
        return ret + bytes([(256-checksum) & 0xFF])

    def get_packet(self):
        """extract packet from internal buffer (if available otherwise return None"""
        data = self._buf
        if len(data) < 3:
            return None
        high, mid, low = data[:3]  # 24bit packet length (big endian int)
        assert high == 0, high  # all messages < 65535 bytes
        size = 256 * mid + low + 3  # counting also 3 bytes of packet length header
        if len(data) < size:
            return None
        ret, self._buf = data[:size], data[size:]
        checksum = sum(ret) & 0xFF
        assert checksum == 0, checksum  # checksum error
        return ret

    def parse_packet(self, data):
        """
        Parse cortexpilot sensors status message
        """
        # expects already validated single sample with 3 bytes length prefix
        #   and checksum at the end
        high, mid, low = data[:3]
        assert high == 0, high  # fixed packet size 2*256+89 bytes
        assert mid*256 + low == 121, (mid, low)
        addr, cmd = data[3:5]
        assert addr == 1, addr
        assert cmd == 0xE, cmd
        offset = 5  # payload offset

        # 4 byte Flags (unsigned long)  0
        #   bit 0 -> 1 = BigRedSwitch
        #   bit 1 -> 1 = MissionButton
        #   bit 2 -> copy of EnableRun flag (motors enabled)
        #   bit 3 -> 1 = Payload loaded, 0 = Payload unloaded - payload indicator
        #   bit 4 -> 1 = LIDAR ScanValid
        #   bit 5 -> 1 = Manual override
        # 4 byte SystemVoltage (float)  4 - battery level for control electronics [V]
        # 4 byte PowerVoltage (float)   8 - battery level for motors [V]
        self.flags, system_voltage, power_voltage = struct.unpack_from('<Iff', data, offset)
        self.lidar_valid = (self.flags & 0x10) == 0x10
        self.emergency_stop = (self.flags & 0x01) == 0x01
        self.bus.publish('emergency_stop', self.emergency_stop)

        self.voltage = [system_voltage, power_voltage]
        self.bus.publish('voltage', [int(v*100) for v in self.voltage])

        # skipped parsing of:
        # 4 byte SpeedM1 (float)       12 - normalized motor M1 (R) speed <-1.0 1.0>
        # 4 byte SpeedM2 (float)       16 - normalized motor M2 (L) speed <-1.0 1.0>
        motors = struct.unpack_from('<ff', data, offset + 12)
        if self.verbose:
            dt = None
            if self.last_processed_cmd is not None and self.last_processed_cmd[-1] is not None:
                dt = (self.time - self.last_processed_cmd[-1]).total_seconds()
            if (self.last_processed_cmd is not None and
                    abs(motors[0]) + abs(motors[1]) < 0.01 and
                    abs(self.last_processed_cmd[0]) + abs(self.last_processed_cmd[1]) > 0.01):
                self.watchdog_count += 1
                if self.min_watchdog_dt is None:
                    self.min_watchdog_dt = dt
                else:
                    self.min_watchdog_dt = min(dt, self.min_watchdog_dt)
            print(self.time, 'Motors', motors, self.watchdog_count, f'dt={dt} min={self.min_watchdog_dt}')

        # skipped parsing of:
        # 4 byte ActualDir (float)     20 - normalized direction for PID controller
        
        # 4 byte EncM1 (signed long)   24 - incremental encoders count for motor M1 (R) since last reset
        # 4 byte EncM2 (signed long)   28 - incremental encoders count for motor M2 (L) since last reset
        encoders = struct.unpack_from('<II', data, offset + 24)

        # skipped parsing of:
        # 1 byte GPS_Valid             32 - 1 = valid data from GPS module
        # 1 byte GPS_Fix               33 - 0 = no fix, 1 = 2D fix, 2 = 3D fix 
        # 4 byte GPS_UTCDate (ulong)   34 - GPS date in YYMMDD format
        # 4 byte GPS_UTCTime (ulong)   38 - GPS time in HHMMSS format
        # 4 byte GPS_Lat (ulong)       42 - format Lat * 1E7
        # 4 byte GPS_Lon (ulong)       46 - format Lon * 1E7
        # 4 byte GPS_Brg (float)       50 - GPS Bearing <0 .. 359> deg

        # 4 byte AHRS_q0 (float)       54 - Orientation Quaternion
        # 4 byte AHRS_q1 (float)       58 - 
        # 4 byte AHRS_q2 (float)       62 - 
        # 4 byte AHRS_q3 (float)       66 - 

        qw, qx, qy, qz = struct.unpack_from('<ffff', data, offset+54)
        orientation = qx, qy, qz, qw
        # identity quat points to north, we need it to point to east
        orientation = quaternion.multiply(orientation, [0, 0, 0.7071068, 0.7071068])
        # correct roll axis by 1.7 degrees
        self.orientation = quaternion.multiply(orientation, [0.0148348, 0, 0, 0.99989])
        self.bus.publish('orientation', list(self.orientation))

        q1, q2, q3, q0 = self.orientation  # quaternion
        #print(self.time, "{:.4f} {:.4f} {:.4f} {:.4f}".format(q0, q1, q2, q3))
        ax =  math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
        ay =  math.asin(2*(q0*q2-q3*q1))
        az =  math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
        # rotation Euler angles are yaw, pitch and roll
        #print(self.time, "{:.4f} {:.4f} {:.4f}".format(math.degrees(az), math.degrees(ay), math.degrees(ax)))
        self.bus.publish('rotation', [round(math.degrees(angle)*100) for angle in [az, ay, ax]])

        # 4 byte Yaw (float)           70 - Heading (Yaw) - machine orientation to magnetic north <0 .. 359> deg
        self.yaw = struct.unpack_from('<f', data, offset + 70)[0]
        #print(math.degrees(x), math.degrees(y), math.degrees(z), self.yaw)
        # 4 byte AccelX (float)        74
        # 4 byte AccelY (float)        78
        # 4 byte AccelZ (float)        82
        # 4 byte GyroX (float)         86
        # 4 byte GyroY (float)         90
        # 4 byte GyroZ (float)         94
        # 4 byte MagX (float)          98
        # 4 byte MagY (float)          102
        # 4 byte MagZ (float)          106
        # 4 byte SystemTick (ulong) 110  - Uptime in milisecond
        uptime = struct.unpack_from('<I', data, offset + 110)[0]
        if self.uptime is not None:
            uptime_diff = uptime - self.uptime
        self.uptime = uptime

        if self.last_encoders is not None:
            step = [sint32_diff(x, prev) for x, prev in zip(encoders, self.last_encoders)]
            self.publish('encoders', step)

#            dist = ENC_SCALE * sum(step)/len(step)
#            angle = ENC_SCALE * (step[0] - step[1])/WHEEL_DISTANCE
            # Skiddy has left 
            dist = ENC_SCALE * (step[0] - step[1])/2.0  # right forward, left negative
            angle = ENC_SCALE * (step[0] + step[1])/WHEEL_DISTANCE
            x, y, heading = self.pose
            # advance robot by given distance and angle
            if abs(angle) < 0.0000001:  # EPS
                # Straight movement - a special case
                x += dist * math.cos(heading)
                y += dist * math.sin(heading)
                #Not needed: heading += angle
            else:
                # Arc
                r = dist / angle
                x += -r * math.sin(heading) + r * math.sin(heading + angle)
                y += +r * math.cos(heading) - r * math.cos(heading + angle)
                heading += angle # not normalized
            self.pose = (x, y, heading)
            self.send_pose()
        self.last_encoders = encoders

        if cmd == 0xE:
            # lidar is not supported
            return

        # 4 byte LidarTimestamp (ulong) 114  - Value of SystemTick when lidar scan was received
        lidar_timestamp = struct.unpack_from('<I', data, offset + 114)[0]
        lidar_diff = lidar_timestamp - self.lidar_timestamp
        self.lidar_timestamp = lidar_timestamp
        if lidar_diff > 150 and self.lidar_valid:
            print(self.time, "lidar invalid:", lidar_diff)
            self.lidar_valid = False
        if lidar_diff != 0 and self.lidar_valid:
            # laser
            # 480 byte Lidar_Scan (ushort) 118 - 239 two-bytes distances from Lidar <0 .. 65535> in [cm]
            # Scan is whole 360 deg with resolution 1.5 deg
            scan = struct.unpack_from('<' + 'H'*239, data, offset + 118)  # TODO should be 240

            # restrict scan only to 270 degrees - cut 1/8th on both sides
#            scan = scan[30:-30]
#            zero_sides = 20
            scan = [10 * d for d in reversed(scan)]  # scale to millimeters
#            scan[:zero_sides] = [0]*zero_sides
#            scan[-zero_sides:] = [0]*zero_sides
            self.publish('scan', scan)

    def process_packet(self, packet):
        assert packet is not None
        if len(packet) < 100:  # TODO cmd value
            print(packet)
        else:
            prev = self.flags
            self.parse_packet(packet)
            if prev != self.flags:
                print(self.time, 'Flags:', hex(self.flags))

    def on_desired_speed(self, data):
        self.desired_speed, self.desired_angular_speed = data[0] / 1000.0, math.radians(data[1] / 100.0)
        """
        if abs(self.desired_speed) < 0.2 and abs(self.desired_angular_speed) > 0.2:
            if self.speeds.__name__ != "oscilate":
                self.speeds = self.oscilate()
        else:
            if self.speeds.__name__ == "oscilate":
                self.speeds = self.plain_speeds()
                """
        self.cmd_flags |= 0x02  # PWM ON

    def run(self):
        try:
            now = self.publish('raw', self.query_version())
            dropped = 0
            while True:
                dt, channel, data = self.listen()
                self.time = dt
                if channel == 'raw':
                    self._buf += data
                    prev = None
                    while True:
                        packet = self.get_packet()
                        if packet is None:
                            if dt > now:
                                packet = prev
                                if packet is not None:
                                    dropped -= 1
                            break
                        prev = packet
                        dropped += 1
                    if packet is not None:
                        self.publish('dropped', dropped)
                        dropped = 0
                        self.process_packet(packet)
                        now = self.publish('raw', self.create_packet())
                elif channel == 'desired_speed':
                    self.on_desired_speed(data)
                else:
                    assert False, channel  # unsupported input channel
        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
