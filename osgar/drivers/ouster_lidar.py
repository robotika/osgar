"""
  Ouster lidar drivers
"""

import logging
import json

from osgar.node import Node
from osgar.bus import BusShutdownException


g_logger = logging.getLogger(__name__)


class OusterLidarDummy(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('http_request')
        self.lidar_url = config["lidar_url"]
        self.config_params = config.get("config_params")
        self.headers = {'Accept': 'application/json', 'Content-Type': 'application/json'}
        self.configuration_status = 0

    def send_conf_params(self):
        data = json.dumps(self.config_params).encode('utf-8')
        self.publish("http_request", [self.lidar_url, self.headers, data])

    def request_configuration(self):
        self.publish("http_request", self.lidar_url)

    def process_udp(self, packet):
        pass
        # print(len(packet))

    def update(self):
        timestamp, channel, data = self.bus.listen()
        if channel == "udp_packet":
            self.process_udp(data)
            if self.configuration_status == 0:
                if self.config_params:
                    self.send_conf_params()
                    self.configuration_status = 1
                else:
                    self.request_configuration()
                    self.configuration_status = 3
        elif channel == "response":
            print(data)
            assert self.configuration_status != 0
            self.request_configuration()
