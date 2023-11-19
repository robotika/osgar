"""
  Support for commercially available outdoor delivery platform FR-07 Pro from company Yuhesen
"""
import struct

from osgar.node import Node


class FR07(Node):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('can', 'emergency_stop')
        self.last_steering = None
        self.last_speed = None
        self.last_emergency_stop = None
        self.last_vehicle_mode = None
        self.last_error_status = None
        self.counters = {}

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
                assert self.counters[msg_id] == payload[-2], (self.counters[msg_id], payload[-2])

            # checksum
            xor = payload[0] ^ payload[1] ^ payload[2] ^ payload[3] ^ payload[4] ^ payload[5] ^ payload[6]
            assert xor == payload[7], (xor,  payload[7])

        if msg_id == 0x18c4d2ef:  # Chassis control feedback command
            # 0100e0ff0f20 d0e1
            target_gear = payload[0] & 0xF
            assert target_gear == 1, target_gear  # P
            speed = ((payload[0] & 0xF0) << 8) + (payload[1] << 4) + (payload[2] & 0x0F)  # 0.001 m/s
            if self.last_speed != speed:
#                print(self.time, f'speed = {speed}')
                self.last_speed = speed
            steering = ((payload[2] & 0xF0) << 8) + (payload[3] << 4) + (payload[4] & 0x0F)  # 0.01 deg
            if self.last_steering != steering:
#                print(self.time, f'steering = {steering}')
                self.last_steering = steering
            vehicle_mode = (payload[5] & 0x30) >> 4
            assert vehicle_mode in [1, 2], vehicle_mode  # 1=remote, 2=stop  (waiting for 0=auto)
            if self.last_vehicle_mode != vehicle_mode:
                print(self.time, f'Vehicle mode: {vehicle_mode}')
                self.last_vehicle_mode = vehicle_mode
        elif msg_id == 0x18c4d7ef:  # Left rear wheel information feedback
            pass
        elif msg_id == 0x18c4d8ef:  # Right rear wheel information feedback
            pass

        elif msg_id == 0x18c4daef:  # Chassis I/O status feedback
            assert payload[0] == 0, payload.hex()  # I/O control enabling status feedback  1=on, 0=off
            assert payload[1] in [0x00, 0x10, 0x20, 0x24, 0x28, 0x30, 0x34, 0x38], payload.hex()  # lights
            assert payload[2] in [0, 1], payload.hex()  # Loudspeaker
            assert payload[3] == 0, payload.hex()  # bumpers
            assert payload[5] == 0, payload.hex()  # enforced charging

        elif msg_id == 0x18c4dcef:  # MCU driver (not documented)
            pass

        elif msg_id == 0x18c4deef:  # Chassis speedometer feedback
            assert payload[:-2].hex() == '000000000000', payload.hex()

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
