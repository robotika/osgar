"""Experimental node for testing shutdown on a timer tick."""

import logging

from osgar.node import Node


class ShutdownTester(Node):
    def on_tick(self, data):
        if data > 3.0:
            logging.getLogger(__name__).info('Requesting stop at tick %.3f', data)
            self.bus.request_stop()
