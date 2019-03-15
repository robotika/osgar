import unittest
from array import array

from osgar.drivers.logusb import LogUSB
from osgar.lib.serialize import serialize, deserialize


class LogUSBTest(unittest.TestCase):

    def test_array(self):
        # USB read returns array('B', [...]), which fails to serialize
        arr = array('B', [1, 2, 3])
        b_arr = serialize(bytes(arr))
        self.assertEqual(deserialize(b_arr), bytes([1, 2, 3]))

# vim: expandtab sw=4 ts=4
