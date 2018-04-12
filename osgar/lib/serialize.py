"""
  serialization of messages
"""
from ast import literal_eval


def serialize(data):
    try:
        bytes_data = data.tobytes()
    except AttributeError:
        if isinstance(data, bytes):
            bytes_data = data
        else:
            bytes_data = bytes(str(data), encoding='ascii')
    return bytes_data


def deserialize(bytes_data):
    return literal_eval(bytes_data.decode('ascii'))

