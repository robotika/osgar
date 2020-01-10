"""
  serialization of messages
"""
from io import BytesIO

import msgpack
import zlib

try:
    import numpy
except:
    numpy = False

MSGPACK_ZLIB_CODE = 42
MSGPACK_NUMPY_CODE = 43


def serialize(data, compress=False):
    if numpy and isinstance(data, (numpy.ndarray, numpy.generic)):
        with BytesIO() as b:
            numpy.save(b, data, allow_pickle=False)
            data = b.getvalue()
        data = zlib.compress(data)
        data = msgpack.ExtType(MSGPACK_NUMPY_CODE, data)
    elif compress:
        packed_data = zlib.compress(msgpack.packb(data, use_bin_type=True))
        data = msgpack.ExtType(MSGPACK_ZLIB_CODE, packed_data)

    return msgpack.packb(data, use_bin_type=True)


def deserialize(bytes_data):
    data = msgpack.unpackb(bytes_data, raw=False)
    if isinstance(data, msgpack.ExtType):
        if data.code == MSGPACK_ZLIB_CODE:
            assert data.code == MSGPACK_ZLIB_CODE, data.code
            data = zlib.decompress(data.data)
            data = msgpack.unpackb(data, raw=False)
        elif data.code == MSGPACK_NUMPY_CODE:
            if not numpy:
                raise TypeError("TypeError: can not deserialize 'numpy.ndarray' object")
            data = zlib.decompress(data.data)
            data = numpy.load(BytesIO(data), allow_pickle=False)
    return data

# vim: expandtab sw=4 ts=4
