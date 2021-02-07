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
        self.debug_arr = []
        self.verbose = False
        self.frame_index = 1  # save as "frame0001.bin"
        self.tasks = [
            [1, b'streamon'],
            [2, b'takeoff'],
            [10, b'cv 360'],
            [20, b'land'],
            [22, b'streamoff']
        ]
        self.last_cmd = None

    def on_cmd_ack(self, data):
        print(self.time, '---', self.last_cmd, data)
        self.last_cmd = None

    def on_status(self, data):
        s = data.strip().split(b';')
        # example b'bat:83'
        for item in s:
            if item.startswith(b'bat:'):
                prev = self.battery
                self.battery = int(item[4:])
                if prev != self.battery:
                    print(self.time, 'Battery:', prev, '->', self.battery)
            elif item.startswith(b'tof:'):
                if self.verbose:
                    self.debug_arr.append((self.time.total_seconds(), int(item[4:])))

    def on_video(self, data):
        return  # hack
        assert len(data) <= 1460, len(data)
        self.buf += data
        print(len(data))
        if len(data) < 1460:
            print(len(self.buf))
            with open('out/frame%04d.bin' % self.frame_index, 'wb') as f:
                f.write(self.buf)
            self.frame_index += 1
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
                    if self.time.total_seconds() > self.tasks[0][0] and self.last_cmd is None:
                        self.last_cmd = self.tasks[0][1]
                        print(self.time, 'SEND', self.last_cmd)
                        self.publish('cmd', self.last_cmd)
                        self.tasks = self.tasks[1:]
        except BusShutdownException:
            pass

    def draw(self):
        import matplotlib.pyplot as plt
        t = [x for x, y in self.debug_arr]
        tof = [y/100.0 for x, y in self.debug_arr]
        plt.plot(t, tof, '-o', linewidth=2)
        plt.xlabel('time (s)')
        plt.show()

# vim: expandtab sw=4 ts=4
