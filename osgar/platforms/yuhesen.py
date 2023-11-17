"""
  Support for commercially available outdoor delivery platform FR-07 Pro from company Yuhesen
"""
import struct

from osgar.node import Node


class FR07(Node):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('can', 'emergency_stop')

    def on_can(self, data):
        msg_id, payload, msg_type = data
        if msg_id == 0x18c4d2ef:  # Chassis control feedback command
            # 0100e0ff0f20 d0e1
            target_gear = payload[0] & 0xF
            assert target_gear == 1, target_gear  # P
            speed = ((payload[0] & 0xF0) << 8) + (payload[1] << 4) + (payload[2] & 0x0F)  # 0.001 m/s
            steering = ((payload[2] & 0xF0) << 8) + (payload[3] << 4) + (payload[4] & 0x0F)  # 0.01 deg
            vehicle_mode = (payload[5] & 0x30) >> 4
            assert vehicle_mode == 2, vehicle_mode  # stop
        elif msg_id == 0x18c4d7ef:  # Left rear wheel information feedback
            pass
        elif msg_id == 0x18c4d8ef:  # Right rear wheel information feedback
            pass
        elif msg_id == 0x18c4daef:  # Chassis I/O status feedback
            pass
        elif msg_id == 0x18c4dcef:  # MCU driver (not documented)
            pass
        elif msg_id == 0x18c4deef:  # Chassis speedometer feedback
            pass
        elif msg_id == 0x18c4e1ef:  # Battery BMS information feedback
            assert payload[:-2].hex() == '000000000000', payload.hex()
        elif msg_id == 0x18c4e2ef:  # Battery BMS mark status feedback
            assert payload[:-2].hex() == '000000000000', payload.hex()
        elif msg_id == 0x18c4eaef:  # Vehicle fault status feedback
            assert payload[:-3].hex() == '3200000000', payload.hex()
            emergency_stop = payload[5] & 0x20 == 0x20
            self.publish('emergency_stop', emergency_stop)
            # 320000000050/70 - fault level 0x2
            # Auto (IO) control CAN communication error = 0x3

            # BMS CAN communication disconnection fault - 44
            # Emergency stop fault - 45
            # Remote controller close alarm - 46
            # Remote controller receiver disconnection fault - 47
        else:
            assert 0, hex(msg_id)  # not supported CAN message ID
