"""
  Universal Terminator - minimal logging and wait for end of STOP
"""
from osgar.node import Node
from osgar.followme import EmergencyStopException


class Terminator(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.raise_exception_on_stop = True

    def on_terminate_if_true(self, data):
        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def on_terminate_if_false(self, data):
        if self.raise_exception_on_stop and not data:
            raise EmergencyStopException()

# vim: expandtab sw=4 ts=4
