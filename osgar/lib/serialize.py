"""
  serialization of messages
"""
#import msgpack


def serialize(data):
    return bytes(str(data), encoding='ascii')
#    return msgpack.packb(data, use_bin_type=True)


def deserialize(bytes_data):
    return eval(bytes_data)
#    return msgpack.unpackb(bytes_data, raw=False)

