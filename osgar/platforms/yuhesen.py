"""
  Support for commercially available outdoor delivery platform FR-07 Pro from company Yuhesen
"""
import math
import struct
from enum import Enum

from osgar.node import Node


WHEEL_DISTANCE = 0.645  # meters left and right rear wheel

class Gear(Enum):
    PARK = 1
    REVERSE = 2
    NEUTRAL = 3
    DRIVE = 4


class FR07(Node):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('can', 'emergency_stop', 'pose2d')
        self.last_steering = None
        self.last_speed = None
        self.last_emergency_stop = None
        self.last_vehicle_mode = None
        self.last_error_status = None
        self.last_bumpers = None
        self.last_left_speed = None
        self.last_right_speed = None
        self.pose = 0, 0, 0
        self.pose_counter = 0
        self.counters = {}

        self.desired_gear = Gear.DRIVE
        self.desired_speed = 0.0  # m/s
        self.desired_steering_angle_deg = 0.0  # degrees
        self.debug_arr = []

    def publish_pose2d(self, left, right):
        dt = 0.04  # 25Hz

        x, y, heading = self.pose

        metricL = left * dt
        metricR = right * dt

        dist = (metricL + metricR)/2.0
        angle = (metricR - metricL)/WHEEL_DISTANCE

        # advance robot by given distance and angle
        if abs(angle) < 0.0000001:  # EPS
            # Straight movement - a special case
            x += dist * math.cos(heading)
            y += dist * math.sin(heading)
            # Not needed: heading += angle
        else:
            # Arc
            r = dist / angle
            x += -r * math.sin(heading) + r * math.sin(heading + angle)
            y += +r * math.cos(heading) - r * math.cos(heading + angle)
            heading += angle  # not normalized
        self.pose = (x, y, heading)
        self.publish('pose2d', [round(x*1000), round(y*1000), round(math.degrees(heading)*100)])

    def on_can(self, data):
        msg_id, payload, msg_type = data

        if msg_id == 0x0:  # During boot-up
            print('BOOT', msg_id, payload.hex(), msg_type)
            return

        # all FR-07 messages use the full CAN message length
        assert len(payload) == 8, len(payload)
        if msg_id != 0x18C4DEEF:  # "Chassis speedometer feedback" does not have the counter
            # running counters (i.e. no lost messages)
            if msg_id not in self.counters:
                self.counters[msg_id] = payload[-2]
            else:
                self.counters[msg_id] = (self.counters[msg_id] + 16) & 0xF0  # upper nybble
                if self.counters[msg_id] != payload[-2]:
                    print(self.time, '!!! LOST PACKET !!!', hex(msg_id), self.counters[msg_id], payload[-2])
                    self.counters[msg_id] = payload[-2]

            # checksum
            xor = payload[0] ^ payload[1] ^ payload[2] ^ payload[3] ^ payload[4] ^ payload[5] ^ payload[6]
            assert xor == payload[7], (xor,  payload[7])

        if msg_id == 0x18c4d2ef:  # Chassis control feedback command
            # 0100e0ff0f20 d0e1
            target_gear = payload[0] & 0xF
            assert target_gear in [1, 2, 3, 4], target_gear  # P, R, N, D
            speed = ((payload[0] & 0xF0) >> 4) + (payload[1] << 4) + ((payload[2] & 0x0F) << 12)  # 0.001 m/s
            if self.last_speed != speed:
#                print(self.time, f'speed = {speed}')
                self.last_speed = speed
            uint16_steering = ((payload[2] & 0xF0) >> 4) + (payload[3] << 4) + ((payload[4] & 0x0F) << 12)  # 0.01 deg
            steering = struct.unpack('<h', struct.pack('H', uint16_steering))[0]
            # trigger command in response

            desired_speed_int = int(self.desired_speed * 1000)
            desired_steering_angle_int = int(self.desired_steering_angle_deg * 100)
            cmd = [
                self.desired_gear.value | ((desired_speed_int << 4) & 0xF0),
                (desired_speed_int >> 4) & 0xFF,
                ((desired_speed_int >> 12) & 0xF) | ((desired_steering_angle_int << 4) & 0xF0),
                (desired_steering_angle_int >> 4) & 0xFF,
                (desired_steering_angle_int >> 12) & 0xF,
                0,
                payload[6]  # running counter
            ]
            cmd.append(cmd[0] ^ cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4] ^ cmd[5] ^ cmd[6])
            self.publish('can', [0x18C4D2D0, bytes(cmd), 1])
            if self.last_steering != steering:
#                print(self.time, f'steering = {steering}')
                self.last_steering = steering
            vehicle_mode = (payload[5] & 0x30) >> 4
            assert vehicle_mode in [0, 1, 2], vehicle_mode  # 0=auto, 1=remote, 2=stop
            if self.last_vehicle_mode != vehicle_mode:
                print(self.time, f'Vehicle mode: {vehicle_mode}')
                self.last_vehicle_mode = vehicle_mode
        elif msg_id == 0x18c4d7ef:  # Left rear wheel information feedback
            left_speed, left_pulse_count = struct.unpack('<hi', payload[:6])
            self.last_left_speed = left_speed/1000.0
            self.pose_counter += 1
            if self.verbose:
                self.debug_arr.append([self.time.total_seconds(), 'left', left_speed/1000.0])
                self.debug_arr.append([self.time.total_seconds(), 'left_pulse', left_pulse_count])
        elif msg_id == 0x18c4d8ef:  # Right rear wheel information feedback
            right_speed, right_pulse_count = struct.unpack('<hi', payload[:6])
            self.last_right_speed = right_speed/1000.0
            self.pose_counter += 1
            if self.verbose:
                self.debug_arr.append([self.time.total_seconds(), 'right', right_speed/1000.0])
                self.debug_arr.append([self.time.total_seconds(), 'right_pulse', right_pulse_count])

        elif msg_id == 0x18c4daef:  # Chassis I/O status feedback
#            assert payload[0] == 0, payload.hex()  # I/O control enabling status feedback  1=on, 0=off
#            assert payload[1] in [0x00, 0x10, 0x20, 0x24, 0x28, 0x30, 0x34, 0x38], payload.hex()  # lights
            assert payload[2] in [0, 1], payload.hex()  # Loudspeaker
            if self.last_bumpers != payload[3]:
                print(self.time, 'Bumpers', payload[3])
                self.last_bumpers = payload[3]
            assert payload[5] == 0, payload.hex()  # enforced charging

            cmd = [
                1,  # I/O control enabled
                0x20,  # lamps (0x20-front)
                1,  # laudspeaker
                0, 0, # reserved
                0,  # enforced power-on flag for charging
                payload[6]  # running counter
            ]
            cmd.append(cmd[0] ^ cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4] ^ cmd[5] ^ cmd[6])
            self.publish('can', [0x18C4D7D0, bytes(cmd), 1])

        elif msg_id == 0x18c4dcef:  # MCU driver (not documented)
            pass

        elif msg_id == 0x18c4deef:  # Chassis speedometer feedback
            milage, accumulated_angle = struct.unpack('<II', payload)
            assert accumulated_angle == 0, (milage, accumulated_angle)  # reserved

        elif msg_id == 0x18c4e1ef:  # Battery BMS information feedback
            assert payload[:-2].hex() == '000000000000', payload.hex()
        elif msg_id == 0x18c4e2ef:  # Battery BMS mark status feedback
            assert payload[:-2].hex() == '000000000000', payload.hex()

        elif msg_id == 0x18c4eaef:  # Vehicle fault status feedback
#            assert payload[:-3].hex() == '3200000000', payload.hex()
            emergency_stop = payload[5] & 0x20 == 0x20
            if self.last_emergency_stop != emergency_stop:
                self.publish('emergency_stop', emergency_stop)
                print(self.time, 'Emergency STOP', emergency_stop)
                self.last_emergency_stop = emergency_stop
            # 320000000050/70 - fault level 0x2
            # Auto (IO) control CAN communication error = 0x3

            # BMS CAN communication disconnection fault - 44
            # Emergency stop fault - 45
            # Remote controller close alarm - 46
            # Remote controller receiver disconnection fault - 47
            error_status = payload[:-3].hex()
            if self.last_error_status != error_status:
                print(self.time, f'Error status: {error_status}')
                self.last_error_status = error_status
        else:
            assert 0, hex(msg_id)  # not supported CAN message ID
        if self.pose_counter >= 8:  # report left & right at 25Hz
            self.pose_counter = 0
            self.publish_pose2d(self.last_left_speed, self.last_right_speed)

    def draw(self):
        import matplotlib.pyplot as plt

        for selection in ['left', 'right']:  # 'left_pulse', 'right_pulse'
            t = [a[0] for a in self.debug_arr if a[1] == selection]
            x = [a[2] for a in self.debug_arr if a[1] == selection]
            line = plt.plot(t, x, '-o', linewidth=2, label=f'speed {selection}')

        plt.xlabel('time (s)')
        plt.ylabel('speed (m/s)')
        plt.legend()
        plt.show()
