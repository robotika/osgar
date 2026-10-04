"""
  RPLidar A1 / A2 / A3 / S1 / C1 2D Laser Scanner Driver
"""
import struct

from osgar.node import Node
from osgar.bus import BusShutdownException


# Protocol constants and commands
SYNC_BYTE = b'\xA5'
SYNC_BYTE2 = b'\x5A'

STOP_CMD = b'\xA5\x25'
RESET_CMD = b'\xA5\x40'
SCAN_CMD = b'\xA5\x20'
FORCE_SCAN_CMD = b'\xA5\x21'
GET_INFO_CMD = b'\xA5\x50'
GET_HEALTH_CMD = b'\xA5\x52'
SET_PWM_BYTE = b'\xF0'

DESCRIPTOR_LEN = 7
SCAN_RESPONSE_LEN = 5
SCAN_TYPE = 0x81

DEFAULT_MOTOR_PWM = 660
MAX_MOTOR_PWM = 1023


def create_pwm_packet(pwm):
    """
    Creates a 6-byte motor PWM control packet.
    """
    assert 0 <= pwm <= MAX_MOTOR_PWM, pwm
    payload = struct.pack("<H", pwm)
    req = SYNC_BYTE + SET_PWM_BYTE + b'\x02' + payload
    checksum = 0
    for v in req:
        checksum ^= v
    return req + bytes([checksum])


def parse_scan_packet(raw):
    """
    Parses a 5-byte scan packet.
    Returns (new_scan, quality, angle_deg, distance_mm) or None if invalid.
    """
    if len(raw) != 5:
        return None
    s = raw[0] & 1
    not_s = (raw[0] >> 1) & 1
    check_bit = raw[1] & 1
    angle_q6 = (raw[1] >> 1) | (raw[2] << 7)

    # Validations:
    # 1. S and !S must be complementary.
    # 2. Check bit must be 1.
    # 3. Angle in degrees must be strictly less than 360.0 (angle_q6 < 360 * 64 = 23040).
    if s == not_s or check_bit != 1 or angle_q6 >= 23040:
        return None

    new_scan = bool(s)
    quality = raw[0] >> 2
    angle = angle_q6 / 64.0
    distance_q2 = raw[3] | (raw[4] << 8)
    distance = distance_q2 / 4.0
    return new_scan, quality, angle, distance


class RPLidar(Node):
    def __init__(self, config, bus):
        bus.register('raw', 'scan')
        super().__init__(config, bus)
        self.buf = b''
        self.scan_size = config.get('scan_size', 360)
        self.min_len = config.get('min_len', 5)
        self.motor_pwm = config.get('motor_pwm', DEFAULT_MOTOR_PWM)
        self.mask = config.get('mask')
        self.blind_zone = config.get('blind_zone')

        self.first_scan = True
        self.synced = False
        self.count = 0
        if self.scan_size is not None:
            self.scan = [0] * self.scan_size
        else:
            self.scan = []

    def apply_mask(self, scan):
        if self.blind_zone is not None:
            scan = [0 if (0 < d < self.blind_zone) else d for d in scan]

        if self.mask is not None:
            assert len(self.mask) == 2, self.mask
            begin, end = self.mask
            assert begin >= 0 and end < 0, (begin, end)
            scan = ([0] * begin) + scan[begin:end] + ([0] * abs(end))
        return scan

    def on_raw(self, data):
        self.buf += data
        while len(self.buf) >= 5:
            # Check for 7-byte scan descriptor: b'\xA5\x5A\x05\x00\x00\x40\x81'
            if self.buf.startswith(SYNC_BYTE + SYNC_BYTE2):
                if len(self.buf) < DESCRIPTOR_LEN:
                    break
                if self.buf[2] == 5 and self.buf[6] == SCAN_TYPE:
                    self.buf = self.buf[DESCRIPTOR_LEN:]
                    self.synced = True
                    continue
                else:
                    self.buf = self.buf[2:]
                    self.synced = False
                    continue

            parsed = parse_scan_packet(self.buf[:5])
            if parsed is None:
                # Synchronization lost or corrupt byte, advance by 1
                self.synced = False
                self.buf = self.buf[1:]
                continue

            if not self.synced:
                if len(self.buf) >= 10:
                    if parse_scan_packet(self.buf[5:10]) is None:
                        # Next 5 bytes do not form a valid packet; current alignment is likely a false positive
                        self.buf = self.buf[1:]
                        continue
                    self.synced = True
                else:
                    self.synced = True

            self.buf = self.buf[5:]
            new_scan, quality, angle, distance = parsed

            if new_scan:
                if not self.first_scan and self.count >= self.min_len:
                    self.publish('scan', self.apply_mask(self.scan))
                self.first_scan = False
                if self.scan_size is not None:
                    self.scan = [0] * self.scan_size
                else:
                    self.scan = []
                self.count = 0

            self.count += 1
            if quality > 0 and distance > 0:
                dist_mm = int(round(distance))
                if self.scan_size is not None:
                    deg = int(angle * self.scan_size / 360.0) % self.scan_size
                    self.scan[deg] = dist_mm
                else:
                    self.scan.append(dist_mm)
            elif self.scan_size is None:
                self.scan.append(0)

    def run(self):
        # 1. Stop scanning in case sensor was previously running
        self.publish('raw', STOP_CMD)
        self.sleep(0.02)
        # 2. Start motor if PWM is configured
        if self.motor_pwm is not None:
            self.publish('raw', create_pwm_packet(self.motor_pwm))
            self.sleep(0.02)
        # 3. Request scan
        self.publish('raw', SCAN_CMD)
        try:
            while True:
                self.update()
        except BusShutdownException:
            pass
        finally:
            try:
                self.publish('raw', STOP_CMD)
                if self.motor_pwm is not None:
                    self.publish('raw', create_pwm_packet(0))
            except Exception:
                pass

    def request_stop(self):
        try:
            self.publish('raw', STOP_CMD)
            if self.motor_pwm is not None:
                self.publish('raw', create_pwm_packet(0))
        except Exception:
            pass
        super().request_stop()

# vim: expandtab sw=4 ts=4
