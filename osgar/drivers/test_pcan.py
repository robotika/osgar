import unittest
from osgar.drivers.pcan import convert_filter_str2hex

class TestPeakCAN(unittest.TestCase):
    def test_convert_filter_str2hex(self):
        filter = [
            {"can_id": "0x0", "can_mask": "0xf", "extended": False}
        ]
        expected_result = [
            {"can_id": 0x0, "can_mask": 0xf, "extended": False}
        ]
        result = convert_filter_str2hex(filter)
        self.assertEqual(result, expected_result)


if __name__ == '__main__':
    unittest.main()
