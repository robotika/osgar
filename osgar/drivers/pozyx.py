"""
  OSGAR Pozyx (utrawide-band trilateration) wrapper
"""
import itertools

import pypozyx

from osgar.node import Node

class Pozyx(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('range')
        serial_port = config['port']
        self.devices = [int(x, 16) for x in config.get('devices', [])]  # unfortunately JSON does not support hex
        self.pozyx = pypozyx.PozyxSerial(serial_port)

    def run(self):
        try:
            device_range = pypozyx.DeviceRange()
            while self.bus.is_alive():
                for from_id, to_id in intertools.combinations(self.devices, 2):
                    status = pozyx.doRanging(from_id, device_range, to_id)
                    self. publish([status, from_id, to_id, [device_range.timestamp, device_range.distance, device_range.RSS]])
        except BusShutdownException:
            pass

    def draw(self):
        """
        Debug Draw
        """
        pass


# vim: expandtab sw=4 ts=4
