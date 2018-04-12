import unittest

from osgar.lib.serialize import serialize, deserialize


class SerializeTest(unittest.TestCase):

    def test_serialization(self):
            self.assertEqual(serialize(b'\x01\x02'), b'\x01\x02')
            position = (51749517, 180462688)
            self.assertEqual(serialize(position), b'(51749517, 180462688)')
            self.assertEqual(serialize((123.4, 'Hi')), b"(123.4, 'Hi')")

# vim: expandtab sw=4 ts=4
