"""
  serialization of messages
"""
import io
import msgpack
import numpy as np


def _default(o):
    if isinstance(o, np.ndarray):
        f = io.BytesIO()
        np.save(f, o)
        return msgpack.ExtType(42, f.getvalue())


def _ext_hook(code, data):
    if code == 42:
        f = io.BytesIO(data)
        o = np.load(f)
        return o
    return msgpack.ExtType(code, data)


def serialize(data):
    return msgpack.packb(data, default=_default, use_bin_type=True)


def deserialize(bytes_data):
    return msgpack.unpackb(bytes_data, ext_hook=_ext_hook, raw=False)

