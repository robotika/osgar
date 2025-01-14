"""
  Ouster lidar drivers
"""

import json

from osgar.node import Node


class OusterLidarUDP(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('http_request', 'lidar_config')
        self.lidar_url = config["lidar_url"]
        self.config_params = config.get("config_params")
        self.headers = {'Accept': 'application/json', 'Content-Type': 'application/json'}
        self.configuration_done = False
        self.configuration_saved = False
        self.verbose = False

    def send_conf_params(self):
        data = json.dumps(self.config_params).encode('utf-8')
        self.publish("http_request", [self.lidar_url, self.headers, data])

    def request_configuration(self):
        self.publish("http_request", self.lidar_url)

    def process_udp(self, packet):
        pass
        # print(len(packet))

    def on_udp_packet(self, data):
        if not self.configuration_done:
            if self.config_params:
                self.send_conf_params()

            self.request_configuration()
            self.configuration_done = True
            return
        if self.configuration_saved:
            self.process_udp(data)

    def on_response(self, data):
        assert self.configuration_done
        assert not self.configuration_saved  # The configuration should be delivered only once.
        if self.verbose:
            print(data)
        self.publish("lidar_config", data)
        self.configuration_saved = True
