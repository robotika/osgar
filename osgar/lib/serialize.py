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
MSGPACK_NUMPY_ZLIB_CODE = 43 # for old logs only, no longer used
MSGPACK_NUMPY_CODE = 44


def serialize(data):
    if numpy and isinstance(data, (numpy.ndarray, numpy.generic)):
        with BytesIO() as b:
            numpy.save(b, data, allow_pickle=False)
            data = b.getvalue()
        data = msgpack.ExtType(MSGPACK_NUMPY_CODE, data)

    return msgpack.packb(data, use_bin_type=True)


def compress(data):
    zlib_data = zlib.compress(data)
    data = msgpack.ExtType(MSGPACK_ZLIB_CODE, zlib_data)
    return msgpack.packb(data, use_bin_type=True)


def deserialize(bytes_data):
    data = msgpack.unpackb(bytes_data, raw=False)
    while isinstance(data, msgpack.ExtType):
        if data.code == MSGPACK_ZLIB_CODE:
            data = zlib.decompress(data.data)
            data = msgpack.unpackb(data, raw=False)
        elif data.code == MSGPACK_NUMPY_ZLIB_CODE: # for old logs only, no longer used
            if not numpy:
                raise TypeError("TypeError: can not deserialize 'numpy.ndarray' object")
            data = zlib.decompress(data.data)
            data = numpy.load(BytesIO(data), allow_pickle=False)
        elif data.code == MSGPACK_NUMPY_CODE:
            if not numpy:
                raise TypeError("TypeError: can not deserialize 'numpy.ndarray' object")
            data = numpy.load(BytesIO(data.data), allow_pickle=False)
    return data

# vim: expandtab sw=4 ts=4
