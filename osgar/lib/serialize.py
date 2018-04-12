"""
  serialization of messages
"""
import msgpack


def serialize(data):
    return msgpack.packb(data, use_bin_type=True)


def deserialize(bytes_data):
    return msgpack.unpackb(bytes_data, raw=False)

