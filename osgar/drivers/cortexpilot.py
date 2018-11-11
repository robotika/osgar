"""
  Driver for robot Robik from cortexpilot.com
"""

import struct
import math

from osgar.node import Node
from osgar.bus import BusShutdownException


# CPR = 9958 (ticks per revolution)
# wheel diameter D = 395 mm
# 1 Rev = 1241 mm
ENC_SCALE = 1.241/9958
WHEEL_DISTANCE = 0.5  # meters TODO confirm


class Cortexpilot(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

        self._buf = b''

        # commands
        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0
        self.cmd_flags = 0x41 #0x40  # 0 = remote steering, PWM OFF, laser ON, TODO

        # status
        self.emergency_stop = None  # uknown state
        self.pose = (0.0, 0.0, 0.0)  # x, y in meters, heading in radians (not corrected to 2PI)
        self.flags = None
        self.voltage = None
        self.last_encoders = None
        self.yaw = None

    def send_pose(self):
        x, y, heading = self.pose
        self.publish('pose2d', [round(x*1000), round(y*1000),
                                round(math.degrees(heading)*100)])

    def query_version(self):
        ret = bytes([0, 0, 3, 0x1, 0x01])
        checksum = sum(ret) & 0xFF
        return ret + bytes([256-checksum])

    def create_packet(self):
        if self.yaw is None:
            self.yaw = 0.0  # hack!
        packet = struct.pack('<ffI', self.desired_speed,
#                             self.desired_angular_speed, self.cmd_flags)
                             self.yaw, self.cmd_flags)
        assert len(packet) < 256, len(packet)  # just to use LSB only
        ret = bytes([0, 0, len(packet) + 2 + 1, 0x1, 0x0C]) + packet
        # addr=0x1, cmd=0xC, length is given by payload, addr, cmd and checksum
        checksum = sum(ret) & 0xFF
        return ret + bytes([256-checksum])

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
        assert high == 0, high  # fixed packet size 2*256+44 bytes
        assert mid == 2, mid
        assert low == 44, low
        addr, cmd = data[3:5]
        assert addr == 1, addr
        assert cmd == 0xC, cmd
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

        if self.last_encoders is not None:
            step = [x - prev for x, prev in zip(encoders, self.last_encoders)]
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

        # laser
        # 2 byte Lidar_LastIdx (ushort)74 - DEPRECATED
        # 480 byte Lidar_Scan (ushort) 76 - 239 two-bytes distances from Lidar <0 .. 65535> in [cm]
        # Scan is whole 360 deg with resolution 1.5 deg
        scan = struct.unpack_from('<' + 'H'*239, data, offset + 76)  # TODO should be 240
        self.publish('scan', [10 * d for d in scan])

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
                            self.parse_packet(packet)
                        self.publish('raw', self.create_packet())
                if channel == 'desired_speed':
                    self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)                    
                    self.cmd_flags |= 0x02  # PWM ON
                    if data == [0, 0]:
                        print("TURN OFF")
                        self.cmd_flags = 0x00  # turn everything OFF (hack for now)

        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
