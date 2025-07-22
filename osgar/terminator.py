"""
  Universal Terminator - minimal logging and wait for end of STOP
"""
from osgar.node import Node
from osgar.followme import EmergencyStopException


class Terminator(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.change_required = config.get('change_required')
        self.prev_state = None  # unknown
        self.raise_exception_on_stop = True

    def on_terminate_if_true(self, data):
        if self.change_required and self.prev_state is None:
            self.prev_state = data
            return  # initial value
        if self.change_required and self.prev_state == data:
            return   # no change
        self.change_required = False  # now it finally changed
        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def on_terminate_if_false(self, data):
        if self.change_required and self.prev_state is None:
            self.prev_state = data
            return  # initial value
        if self.change_required and self.prev_state == data:
            return   # no change
        self.change_required = False  # now it finally changed
        if self.raise_exception_on_stop and not data:
            raise EmergencyStopException()

# vim: expandtab sw=4 ts=4
