"""
  Driver "replay" replaying old logfile
"""

from threading import Thread

from osgar.logger import LogReader, lookup_stream_names
from osgar.lib.serialize import deserialize


class ReplayDriver:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus

        self.filename = config['filename']
        self.pins = config['pins']

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        names = lookup_stream_names(self.filename)
        print(names)
        ids = [i + 1 for i, name in enumerate(names) if name in self.pins]
        print(ids)
        for timestamp, channel_index, data_raw in LogReader(self.filename,
                only_stream_id=ids):
            if not self.bus.is_alive():
                break
            channel = names[channel_index - 1]
            assert channel in self.pins
            data = deserialize(data_raw)
            # TODO reuse timestamp
            self.bus.publish(self.pins[channel], data)
        print('Replay completed!')

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
