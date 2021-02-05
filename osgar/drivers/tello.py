"""
  Driver for a mini drone Tello

  for details see blog at
     https://robotika.cz/robots/paula/
"""

import cv2

from osgar.node import Node
from osgar.bus import BusShutdownException


def save_h264_img(payload):
    tmpFile = open("tmp.bin", "wb")
    tmpFile.write(payload)
    tmpFile.flush()
    tmpFile.close()
    cap = cv2.VideoCapture("tmp.bin")
    ret, frame = cap.read()
    if ret:
        cv2.imwrite("test.jpg", frame)  # dirty solution how to get image from OpenCV to Pygame :(
        assert 0


class TelloDrone(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cmd')
        self.battery = None
        self.buf = b''
        self.tasks = [
            [2, b'streamon'],
            [5, b'streamoff']
        ]

    def on_status(self, data):
        s = data.strip().split(b';')
        # example b'bat:83'
        for item in s:
            if item.startswith(b'bat:'):
                prev = self.battery
                self.battery = int(item[4:])
                if prev != self.battery:
                    print('Battery:', prev, '->', self.battery)

    def on_video(self, data):
        assert len(data) <= 1460, len(data)
        self.buf += data
        print(len(data))
        if len(data) < 1460:
            print(len(self.buf))
            # save_h264_img(self.buf)
            self.buf = b''

    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unknown

    def run(self):
        self.publish('cmd', b'command')
        try:
            while True:
                self.update()
                if len(self.tasks) > 0:
                    if self.time.total_seconds() > self.tasks[0][0]:
                        self.publish('cmd', self.tasks[0][1])
                        self.tasks = self.tasks[1:]
        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
