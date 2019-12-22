"""
  serialization of messages
"""
import msgpack
import zlib

MSGPACK_ZLIB_CODE = 42


def serialize(data, compress=False):
    if compress:
        packed_data = zlib.compress(msgpack.packb(data, use_bin_type=True))
        return msgpack.packb(msgpack.ExtType(MSGPACK_ZLIB_CODE, packed_data), use_bin_type=True)

    return msgpack.packb(data, use_bin_type=True)


def deserialize(bytes_data):
    data = msgpack.unpackb(bytes_data, raw=False)
    if isinstance(data, msgpack.ExtType):
        assert data.code == MSGPACK_ZLIB_CODE, data.code
        data = zlib.decompress(data.data)
        data = msgpack.unpackb(data, raw=False)
    return data

# vim: expandtab sw=4 ts=4
