import unittest
import unittest.mock
import numpy

from osgar.lib.serialize import serialize, deserialize


class SerializeTest(unittest.TestCase):

    def test_serialization(self):
        self.assertEqual(serialize(b'\x01\x02'), b'\xc4\x02\x01\x02')
        position = (51749517, 180462688)
        self.assertEqual(serialize(position), b'\x92\xce\x03\x15\xa2\x8d\xce\n\xc1\xa4`')
        self.assertEqual(serialize((123.4, 'Hi')), b'\x92\xcb@^\xd9\x99\x99\x99\x99\x9a\xa2Hi')

    def test_packed_data(self):
        data = [0]*1000
        packet = serialize(data)
        self.assertEqual(len(packet), 1003)
        self.assertEqual(deserialize(packet), data)

        compressed_packet = serialize(data, compress=True)
        self.assertEqual(len(compressed_packet), 22)
        self.assertEqual(deserialize(compressed_packet), data)

    def test_numpy(self):
        data = numpy.asarray(range(1, 100, 2), dtype=numpy.uint16)
        packet = serialize(data)
        packet_zlib = serialize(data, compress=True)
        self.assertEqual(len(packet), 231)
        self.assertEqual(len(packet_zlib), 163)
        dedata = deserialize(packet)
        dedata_zlib = deserialize(packet_zlib)
        self.assertTrue(numpy.array_equal(dedata, data))
        self.assertEqual(dedata.dtype, data.dtype)
        self.assertTrue(numpy.array_equal(dedata_zlib, data))
        self.assertEqual(dedata_zlib.dtype, data.dtype)

        with unittest.mock.patch('osgar.lib.serialize.numpy', new=False):
            with self.assertRaises(TypeError):
                serialize(data)
            with self.assertRaises(TypeError):
                deserialize(packet)

# vim: expandtab sw=4 ts=4
