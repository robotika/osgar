"""
  Send GO commands via ZeroMQ
"""
import time

import zmq

from osgar.drivers.rosproxy import packCmdVel, prefix4BytesLen

context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.PUSH)
socket.connect("tcp://localhost:5556")

while True:
    msg = prefix4BytesLen(packCmdVel(1.0, 0.0))
    socket.send(msg)
    time.sleep(0.1)
