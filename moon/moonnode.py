"""
  MoonNode - parent for Moon processing nodes
"""
from datetime import timedelta
from random import Random

from osgar.node import Node

# project constants
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOCAL_LENGTH = 381
CAMERA_BASELINE = 0.42 # meters
WHEEL_RADIUS = 0.275  # meters
WHEEL_SEPARATION_WIDTH = 1.87325  # meters
WHEEL_SEPARATION_LENGTH = 1.5748  # meters
LIDAR_BEAM_SPACING = 0.0262626260519 # angle difference in radians between two consecutive LIDAR beams

class MoonNode(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("request")

        self.rand = Random(0)
        self.sim_time = None
        self.monitors = []
        self.requests = {}
        self.debug = config.get('debug', False)

    def on_sim_clock(self, data):
        self.sim_time = timedelta(seconds=data[0], microseconds=data[1])

    def on_response(self, data):
        token, response = data
        #print(self.sim_time, "controller:response received: token=%s, response=%s" % (token, response))
        if token in self.requests.keys():
            callback = self.requests[token]
            self.requests.pop(token)
            if callback is not None:
                callback(response)

    def send_request(self, cmd, callback=None):
        """Send ROS Service Request from a single place"""
        token = hex(self.rand.getrandbits(128))
        self.requests[token] = callback
        #print(self.sim_time, "controller:send_request:token: %s, command: %s" % (token, cmd))
        self.publish('request', [token, cmd])

    def update(self):
        channel = super().update()

        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

        for m in self.monitors:
            m(self, channel)

        return channel

# vim: expandtab sw=4 ts=4
