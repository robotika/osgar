"""
  Driver for robot Robik from cortexpilot.com
"""

import ctypes
import struct
import math

from osgar.node import Node
from osgar.bus import BusShutdownException


# CPR = 9958 (ticks per revolution)
# wheel diameter D = 395 mm
# 1 Rev = 1241 mm
ENC_SCALE = 1.241/9958
WHEEL_DISTANCE = 0.88  # meters TODO confirm
RAMP_STEP = 0.1  # fractional number for speed in -1.0 .. 1.0


def sint32_diff(a, b):
    return ctypes.c_int32(a - b).value


class Cortexpilot(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

        self._buf = b''

        # commands
        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0
        self.cmd_flags = 0x40 #0x41  # 0 = remote steering, PWM OFF, laser ON, TODO

        # status
        self.emergency_stop = None  # uknown state
        self.pose = (0.0, 0.0, 0.0)  # x, y in meters, heading in radians (not corrected to 2PI)
        self.flags = None
        self.voltage = None
        self.last_encoders = None
        self.yaw = None
        self.lidar_valid = False
        self.last_speed_cmd = 0.0
        self.uptime = None

    def send_pose(self):
        x, y, heading = self.pose
        self.publish('pose2d', [round(x*1000), round(y*1000),
                                round(math.degrees(heading)*100)])

    def query_version(self):
        ret = bytes([0, 0, 3, 0x1, 0x01])
        checksum = sum(ret) & 0xFF
        return ret + bytes([(256-checksum) & 0xFF])

    def create_packet(self):
        if self.yaw is None:
            self.yaw = 0.0  # hack!

        speed_frac = self.desired_speed
        if not self.lidar_valid:
            speed_frac = 0.0
        if speed_frac > self.last_speed_cmd + RAMP_STEP:
            speed_frac = self.last_speed_cmd + RAMP_STEP
        elif speed_frac < self.last_speed_cmd - RAMP_STEP:
            speed_frac = self.last_speed_cmd - RAMP_STEP
        self.last_speed_cmd = speed_frac

        packet = struct.pack('<ffI', speed_frac,
                             -self.desired_angular_speed, self.cmd_flags)  # Robik has positive to the right
#                             self.yaw, self.cmd_flags)
        assert len(packet) < 256, len(packet)  # just to use LSB only
        ret = bytes([0, 0, len(packet) + 2 + 1, 0x1, 0x0D]) + packet
        # addr=0x1, cmd=0xD, length is given by payload, addr, cmd and checksum
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
        assert mid == 2, mid
        assert low == 89, low
        addr, cmd = data[3:5]
        assert addr == 1, addr
        assert cmd == 0xD, cmd
        offset = 5  # payload offset

        # 4 byte Flags (unsigned long)  0
        #   bit 0 -> 1 = BigRedSwitch
        #   bit 1 -> 1 = MissionButton
        #   bit 2 -> copy of EnableRun flag (motors enabled)
        #   bit 3 -> 1 = Payload loaded, 0 = Payload unloaded - payload indicator
        #   bit 4 -> 1 = LIDAR ScanValid
        #   bit 5 -> 1 = Manual override
        # 4 byte SystemVoltage (float)  4 - battery level for control electronics [V]
        self.flags, self.voltage = struct.unpack_from('<If', data, offset)
        self.lidar_valid = (self.flags & 0x10) == 0x10

        # skipped parsing of:
        # 4 byte PowerVoltage (float)   8 - battery level for motors [V]

        # 4 byte SpeedM1 (float)       12 - normalized motor M1 (R) speed <-1.0 1.0>
        # 4 byte SpeedM2 (float)       16 - normalized motor M2 (L) speed <-1.0 1.0>
        motors = struct.unpack_from('<ff', data, offset + 12)

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

        # 4 byte Yaw (float)           70 - Heading (Yaw) - machine orientation to magnetic north <0 .. 359> deg
        self.yaw = struct.unpack_from('<f', data, offset + 70)[0]
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
        # 4 byte LidarTimestamp (ulong) 114  - Value of SystemTick when lidar scan was received
        # 480 byte Lidar_Scan          118

        if self.last_encoders is not None:
            step = [sint32_diff(x, prev) for x, prev in zip(encoders, self.last_encoders)]
            self.publish('encoders', step)

            dist = ENC_SCALE * sum(step)/len(step)
            angle = ENC_SCALE * (step[0] - step[1])/WHEEL_DISTANCE
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

        if self.lidar_valid:
            # laser
            # 4 byte LidarTimestamp (unsigned long) 114  - Value of SystemTick when lidar scan was received
            # 480 byte Lidar_Scan (ushort) 118 - 239 two-bytes distances from Lidar <0 .. 65535> in [cm]
            # Scan is whole 360 deg with resolution 1.5 deg
            scan = struct.unpack_from('<' + 'H'*239, data, offset + 118)  # TODO should be 240

            # restrict scan only to 270 degrees - cut 1/8th on both sides
            scan = scan[30:-30]
            zero_sides = 20
            scan = [10 * d for d in reversed(scan)]
            scan[:zero_sides] = [0]*zero_sides
            scan[-zero_sides:] = [0]*zero_sides
            self.publish('scan', scan)  # scale to millimeters

    def run(self):
        try:
            self.publish('raw', self.query_version())
            while True:
                dt, channel, data = self.listen()
                self.time = dt
                if channel == 'raw':
                    self._buf += data
                    packet = self.get_packet()
                    if packet is not None:
                        if len(packet) < 256:  # TODO cmd value
                            print(packet)
                        else:
                            prev = self.flags
                            self.parse_packet(packet)
                            if prev != self.flags:
                                print(self.time, 'Flags:', hex(self.flags))
                        self.publish('raw', self.create_packet())
                if channel == 'desired_speed':
                    self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)                    
                    self.cmd_flags |= 0x02  # PWM ON
#                    if data == [0, 0]:
#                        print("TURN OFF")
#                        self.cmd_flags = 0x00  # turn everything OFF (hack for now)

        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
