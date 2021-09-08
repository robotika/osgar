import io
import numpy as np

from osgar.bus import BusShutdownException
from osgar.node import Node

def serialize(arr):
    buf = io.BytesIO()
    np.save(buf, arr, allow_pickle=False)
    return buf.getvalue()

class Serialize(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

        bus.register(*config.get('channels', []))


    def run(self):
        try:
            while True:
                timestamp, channel, data = self.bus.listen()
                self.publish(channel, serialize(data))
        except BusShutdownException:
            pass


