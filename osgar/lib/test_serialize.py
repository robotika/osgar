import unittest

import numpy as np

from osgar.lib.serialize import serialize, deserialize


class SerializeTest(unittest.TestCase):

    def test_serialization(self):
            self.assertEqual(serialize(b'\x01\x02'), b'\xc4\x02\x01\x02')
            position = (51749517, 180462688)
            self.assertEqual(serialize(position), b'\x92\xce\x03\x15\xa2\x8d\xce\n\xc1\xa4`')
            self.assertEqual(serialize((123.4, 'Hi')), b'\x92\xcb@^\xd9\x99\x99\x99\x99\x9a\xa2Hi')

    def test_numpy(self):
        a = np.ones(3)
        self.assertTrue(np.array_equal(a, deserialize(serialize(a))))

        b = ([3], {1:2}, np.ones(5),)
        bb = deserialize(serialize(b))
        self.assertEqual(b[0], bb[0])
        self.assertEqual(b[1], bb[1])
        self.assertTrue(np.array_equal(b[2], bb[2]))

# vim: expandtab sw=4 ts=4
