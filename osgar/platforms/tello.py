"""
  Driver for a mini drone Tello

  for details see blog at
     https://robotika.cz/robots/paula/
"""
# doc:
#   https://terra-1-g.djicdn.com/2d4dce68897a46b19fc717f3576b7c6a/Tello%20%E7%BC%96%E7%A8%8B%E7%9B%B8%E5%85%B3/For%20Tello/Tello%20SDK%20Documentation%20EN_1.3_1122.pdf

from enum import Enum

import cv2
import av

from osgar.node import Node
from osgar.bus import BusShutdownException

# Available commands - SDK page 3
class TelloCmd(Enum):
    COMMAND = b'command'  # entry SDK mode
    TAKEOFF = b'takeoff'  # Tello auto takeoff
    LAND = b'land'  # Tello auto land
    STREAMON = b'streamon'  # Set video stream on
    STREAMOFF = b'streamoff'  # Set video stream off
    EMERGENCY = b'emergency'  # Stop all motors immediately
    UP = b'up'  # Tello fly up with distance x cm, x: 20-500
    DOWN = b'down'  # Tello fly down with distance x cm, x: 20-500
    LEFT = b'left'  # Tello fly left with distance x cm, x: 20-500
    RIGHT = b'right'  # Tello fly right with distance x cm, x: 20-500
    FORWARD = b'forward'  # Tello fly forward with distance x cm, x: 20-500
    BACK = b'back'  # Tello fly back with distance x cm, x: 20-500
    CW = b'cw'  # Tello rotate x degree clockwise, x: 1-3600
    CCW = b'ccw'  # Tello rotate x degree counter-clockwise, x: 1-3600
    FLIP = b'flip'  # Tello fly flip x, l (left), r (right), f (forward), b (back)
    GO = b'GO'  # go x y z speed - Tello fly to x y z in speed (cm/s) (x: 20-500, y: 20-500, z: 20-500, speed: 10-100)
    CURVE = b'curve'  # curve x1 y1 z1 x2 y2 z2 speed - Tello fly a curve defined by the current and two given coordinates
            # with speed (cm/s) If the arc radius is not within the range of 0.5-10 meters, it responses false
            # x1, x2: 20-500, y1, y2: 20-500, z1, z2: 20-500, speed: 10-60, x/y/z can’t be between -20 – 20 at the same time .
    SPEED = b'speed'  # set speed to x cm/s, x: 10-100
    RC = b'rc'  # rc a b c d - Send RC control via four channels. a: left/right (-100~100), b: forward/backward (-100~100),
            # c: up/down (-100~100), d: yaw (-100~100)


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
        bus.register('cmd', 'jpeg')
        self.codec = av.CodecContext.create('h264', 'r')
        self.battery = None
        self.buf = b''
        self.debug_arr = []
        self.frame_index = 1  # save as "frame0001.bin"
        default_tasks = [
            [1, b'streamon'],
            [2, b'takeoff'],
            [11, b'up 100'],
            [12, b'cw 360'],
            [20, b'land'],
            [22, b'streamoff']
        ]
        tasks = config.get('tasks', default_tasks)
        self.tasks = []
        for time_sec, cmd in tasks:
            if isinstance(cmd, str):
                cmd = cmd.encode('utf-8')
            self.tasks.append([time_sec, cmd])
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
            elif item.startswith(b'yaw:'):
                if self.verbose:
                    print(self.time, item)

    def on_video(self, data):
        try:
            packets = self.codec.parse(data)
            for packet in packets:
                frames = self.codec.decode(packet)
                for frame in frames:
                    img = frame.to_ndarray(format='bgr24')
                    retval, jpeg_data = cv2.imencode('.jpg', img)
                    if retval:
                        self.publish('jpeg', jpeg_data.tobytes())
        except Exception as e:
            print(f"Error decoding video frame: {e}")

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
