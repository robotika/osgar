import unittest
from unittest.mock import MagicMock, call
import struct

from osgar.drivers.rplidar import (
    RPLidar,
    create_pwm_packet,
    parse_scan_packet,
    SYNC_BYTE,
    SYNC_BYTE2,
    STOP_CMD,
    SCAN_CMD,
    SCAN_TYPE,
    DESCRIPTOR_LEN,
)
from osgar.lib.config import get_class_by_name


def make_scan_packet(new_scan, quality, angle_deg, dist_mm):
    """Helper to construct a 5-byte raw scan packet."""
    s = 1 if new_scan else 0
    not_s = 0 if new_scan else 1
    b0 = ((quality & 0x3F) << 2) | (not_s << 1) | s
    angle_q6 = int(round(angle_deg * 64.0))
    b1 = ((angle_q6 & 0x7F) << 1) | 1  # check bit is 1
    b2 = (angle_q6 >> 7) & 0xFF
    dist_q2 = int(round(dist_mm * 4.0))
    b3 = dist_q2 & 0xFF
    b4 = (dist_q2 >> 8) & 0xFF
    return bytes([b0, b1, b2, b3, b4])


class RPLidarTest(unittest.TestCase):

    def test_create_pwm_packet(self):
        # PWM = 0
        pkt0 = create_pwm_packet(0)
        self.assertEqual(len(pkt0), 6)
        self.assertEqual(pkt0, b'\xA5\xF0\x02\x00\x00\x57')

        # PWM = 660 (default)
        pkt660 = create_pwm_packet(660)
        self.assertEqual(pkt660, b'\xA5\xF0\x02\x94\x02\xC1')

        # PWM = 1023 (max)
        pkt1023 = create_pwm_packet(1023)
        self.assertEqual(pkt1023, b'\xA5\xF0\x02\xFF\x03\xAB')

        with self.assertRaises(AssertionError):
            create_pwm_packet(1024)
        with self.assertRaises(AssertionError):
            create_pwm_packet(-1)

    def test_parse_scan_packet_valid(self):
        pkt = make_scan_packet(new_scan=True, quality=15, angle_deg=90.0, dist_mm=1200.0)
        parsed = parse_scan_packet(pkt)
        self.assertIsNotNone(parsed)
        new_scan, quality, angle, dist = parsed
        self.assertTrue(new_scan)
        self.assertEqual(quality, 15)
        self.assertAlmostEqual(angle, 90.0, places=2)
        self.assertAlmostEqual(dist, 1200.0, places=2)

        pkt2 = make_scan_packet(new_scan=False, quality=30, angle_deg=270.5, dist_mm=3456.75)
        parsed2 = parse_scan_packet(pkt2)
        self.assertIsNotNone(parsed2)
        new_scan, quality, angle, dist = parsed2
        self.assertFalse(new_scan)
        self.assertEqual(quality, 30)
        self.assertAlmostEqual(angle, 270.5, places=2)
        self.assertAlmostEqual(dist, 3456.75, places=2)

    def test_parse_scan_packet_invalid(self):
        # Short packet
        self.assertIsNone(parse_scan_packet(b'\x01\x02\x03'))

        # Check bit is 0
        pkt = bytearray(make_scan_packet(new_scan=True, quality=15, angle_deg=90.0, dist_mm=1000.0))
        pkt[1] &= ~0x01  # clear check bit
        self.assertIsNone(parse_scan_packet(bytes(pkt)))

        # S and !S mismatch (both 1)
        pkt[0] |= 0x03
        self.assertIsNone(parse_scan_packet(bytes(pkt)))

        # S and !S mismatch (both 0)
        pkt[0] &= ~0x03
        self.assertIsNone(parse_scan_packet(bytes(pkt)))

        # Angle out of range (>= 360 deg)
        # 360 * 64 = 23040 = 0x5A00
        b1 = ((0x00 & 0x7F) << 1) | 1
        b2 = (23040 >> 7) & 0xFF
        bad_angle_pkt = bytes([0x01, b1, b2, 0x00, 0x00])
        self.assertIsNone(parse_scan_packet(bad_angle_pkt))

    def test_scan_publishing_360(self):
        handler = MagicMock()
        lidar = RPLidar(config={'scan_size': 360, 'min_len': 5}, bus=handler)

        # Descriptor + some partial initial measurements
        desc = b'\xA5\x5A\x05\x00\x00\x40\x81'
        initial_data = desc
        for deg in range(100, 110):
            initial_data += make_scan_packet(new_scan=False, quality=20, angle_deg=float(deg), dist_mm=1500.0)
        lidar.on_raw(initial_data)

        # First new_scan arrives (completing the partial initial scan)
        # Because first_scan was True, partial scan is discarded
        lidar.on_raw(make_scan_packet(new_scan=True, quality=20, angle_deg=0.0, dist_mm=1000.0))
        handler.publish.assert_not_called()

        # Send full revolution for angles 1 to 359
        full_rev = b''
        for deg in range(1, 360):
            full_rev += make_scan_packet(new_scan=False, quality=20, angle_deg=float(deg), dist_mm=1000.0 + deg)
        lidar.on_raw(full_rev)
        handler.publish.assert_not_called()

        # Now start of next revolution triggers publication of the full 360 scan
        lidar.on_raw(make_scan_packet(new_scan=True, quality=20, angle_deg=0.0, dist_mm=2000.0))
        handler.publish.assert_called_once()
        channel, scan = handler.publish.call_args[0]
        self.assertEqual(channel, 'scan')
        self.assertEqual(len(scan), 360)
        self.assertEqual(scan[0], 1000)
        self.assertEqual(scan[100], 1100)
        self.assertEqual(scan[359], 1359)
        self.assertTrue(all(isinstance(x, int) for x in scan))

    def test_zero_distance_and_quality(self):
        handler = MagicMock()
        lidar = RPLidar(config={'scan_size': 360, 'min_len': 5}, bus=handler)

        # First scan trigger
        lidar.on_raw(make_scan_packet(new_scan=True, quality=20, angle_deg=0.0, dist_mm=500.0))

        # Only measure degrees 10, 20, 30; other degrees have quality 0 / dist 0
        stream = b''
        for deg in range(1, 360):
            if deg in (10, 20, 30):
                stream += make_scan_packet(new_scan=False, quality=25, angle_deg=float(deg), dist_mm=800.0)
            else:
                stream += make_scan_packet(new_scan=False, quality=0, angle_deg=float(deg), dist_mm=0.0)
        lidar.on_raw(stream)

        # Next revolution triggers publish
        lidar.on_raw(make_scan_packet(new_scan=True, quality=20, angle_deg=0.0, dist_mm=500.0))
        handler.publish.assert_called_once()
        channel, scan = handler.publish.call_args[0]
        self.assertEqual(scan[10], 800)
        self.assertEqual(scan[20], 800)
        self.assertEqual(scan[30], 800)
        self.assertEqual(scan[15], 0)
        self.assertEqual(scan[100], 0)

    def test_stream_resynchronization(self):
        handler = MagicMock()
        lidar = RPLidar(config={'scan_size': 360, 'min_len': 2}, bus=handler)

        lidar.on_raw(make_scan_packet(new_scan=True, quality=20, angle_deg=0.0, dist_mm=1000.0))

        # Valid packet, then 3 corrupt/garbage bytes, then valid packet stream
        pkt1 = make_scan_packet(new_scan=False, quality=20, angle_deg=10.0, dist_mm=1100.0)
        garbage = b'\xFF\xFE\xFD'
        pkt2 = make_scan_packet(new_scan=False, quality=20, angle_deg=20.0, dist_mm=1200.0)
        pkt3 = make_scan_packet(new_scan=False, quality=20, angle_deg=30.0, dist_mm=1300.0)

        lidar.on_raw(pkt1 + garbage + pkt2 + pkt3)
        # Next revolution
        lidar.on_raw(make_scan_packet(new_scan=True, quality=20, angle_deg=0.0, dist_mm=1000.0))

        handler.publish.assert_called_once()
        scan = handler.publish.call_args[0][1]
        self.assertEqual(scan[10], 1100)
        self.assertEqual(scan[20], 1200)
        self.assertEqual(scan[30], 1300)

    def test_mask_and_blind_zone(self):
        handler = MagicMock()
        config = {
            'scan_size': 360,
            'blind_zone': 500,
            'mask': [2, -2],
            'min_len': 2
        }
        lidar = RPLidar(config=config, bus=handler)

        lidar.on_raw(make_scan_packet(new_scan=True, quality=20, angle_deg=0.0, dist_mm=1000.0))
        # Degree 1 (within mask begin): 1000mm -> masked to 0
        # Degree 10: 400mm (< blind_zone) -> 0
        # Degree 20: 800mm (valid) -> 800
        # Degree 359 (within mask end): 1000mm -> masked to 0
        stream = make_scan_packet(new_scan=False, quality=20, angle_deg=1.0, dist_mm=1000.0)
        stream += make_scan_packet(new_scan=False, quality=20, angle_deg=10.0, dist_mm=400.0)
        stream += make_scan_packet(new_scan=False, quality=20, angle_deg=20.0, dist_mm=800.0)
        stream += make_scan_packet(new_scan=False, quality=20, angle_deg=359.0, dist_mm=1000.0)
        lidar.on_raw(stream)

        lidar.on_raw(make_scan_packet(new_scan=True, quality=20, angle_deg=0.0, dist_mm=1000.0))
        scan = handler.publish.call_args[0][1]
        self.assertEqual(scan[0], 0)
        self.assertEqual(scan[1], 0)
        self.assertEqual(scan[10], 0)
        self.assertEqual(scan[20], 800)
        self.assertEqual(scan[358], 0)
        self.assertEqual(scan[359], 0)

    def test_run_lifecycle_commands(self):
        handler = MagicMock()
        # Mock update to raise BusShutdownException to terminate run loop
        handler.listen.side_effect = SystemExit()
        lidar = RPLidar(config={'motor_pwm': 660}, bus=handler)
        lidar.sleep = MagicMock()

        from osgar.bus import BusShutdownException
        lidar.update = MagicMock(side_effect=BusShutdownException())

        lidar.run()

        # Check commands published in run():
        # 1. STOP_CMD
        # 2. PWM packet (660)
        # 3. SCAN_CMD
        # In finally: STOP_CMD and PWM packet (0)
        published_raw = [call_args[0][1] for call_args in handler.publish.call_args_list if call_args[0][0] == 'raw']
        self.assertIn(STOP_CMD, published_raw)
        self.assertIn(create_pwm_packet(660), published_raw)
        self.assertIn(SCAN_CMD, published_raw)
        self.assertIn(create_pwm_packet(0), published_raw)

    def test_request_stop(self):
        handler = MagicMock()
        lidar = RPLidar(config={'motor_pwm': 660}, bus=handler)
        lidar.request_stop()
        published_raw = [call_args[0][1] for call_args in handler.publish.call_args_list if call_args[0][0] == 'raw']
        self.assertIn(STOP_CMD, published_raw)
        self.assertIn(create_pwm_packet(0), published_raw)

    def test_driver_registration(self):
        cls = get_class_by_name('rplidar')
        self.assertEqual(cls, RPLidar)

    def test_config_load(self):
        import os
        from osgar.lib.config import config_load
        config_path = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'test-rplidar.json')
        cfg = config_load(config_path)
        self.assertIn('robot', cfg)
        self.assertIn('lidar', cfg['robot']['modules'])
        self.assertEqual(cfg['robot']['modules']['lidar']['driver'], 'rplidar')
        self.assertIn('serial', cfg['robot']['modules'])
        self.assertEqual(cfg['robot']['modules']['serial']['driver'], 'serial')


if __name__ == '__main__':
    unittest.main()
