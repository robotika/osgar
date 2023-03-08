import datetime
import io
import math
import struct

from osgar.node import Node

SLIP_END = 0xC0
SLIP_ESCAPE = 0xDB
SLIP_ESCAPED_END = 0xDC
SLIP_ESCAPED_ESCAPE = 0xDD


def slip_stream():
    SLIP_STATE_NORMAL, SLIP_STATE_ESCAPED = list(range(2))
    state = SLIP_STATE_NORMAL
    buf = []
    packets = []
    while True:
        raw = (yield packets)
        packets = []
        for b in list(raw):
            if state == SLIP_STATE_NORMAL:
                if b == SLIP_ESCAPE:
                    state = SLIP_STATE_ESCAPED
                elif b == SLIP_END:
                    if not buf:
                        # Ignore empty frames.
                        continue
                    packets.append(bytes(buf))
                    buf = []
                else:
                    buf.append(b)
            elif b == SLIP_ESCAPED_ESCAPE:
                buf.append(SLIP_ESCAPE)
                state = SLIP_STATE_NORMAL
            elif b == SLIP_ESCAPED_END:
                buf.append(SLIP_END)
                state = SLIP_STATE_NORMAL
            else:
                print('SLIP: Received and discarded a corrupted message.')
                buf = []
                state = SLIP_STATE_NORMAL


class Slip(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('packet', 'raw')
        self.slip_stream = slip_stream()
        next(self.slip_stream)  # Warmimg the generator up.


    def on_packet(self, data):
        buf = io.BytesIO()
        # Flush data on the line.
        buf.write(bytes([SLIP_END]))
        # Place message data.
        for b in data:
            if b == SLIP_END:
                written = [SLIP_ESCAPE, SLIP_ESCAPED_END]
            elif b == SLIP_ESCAPE:
                written = [SLIP_ESCAPE, SLIP_ESCAPED_ESCAPE]
            else:
                written = [b]
            buf.write(bytes(written))
        # Finish the message.
        buf.write(bytes([SLIP_END]))
        self.publish('raw', buf.getvalue())


    def on_raw(self, data):
        packets = self.slip_stream.send(data)
        for packet in packets:
            self.publish('packet', packet)


class Deedee(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cmd', 'pose2d', 'emergency_stop', 'stdout')
        self.emergency_stop_status = None
        self.wheel_base = config.get('wheel_base', 0.298)
        self.max_command_validity = config.get('max_command_validity', 0.8)
        self.control_timeout = datetime.timedelta(seconds=config.get('control_timeout', 1.0))
        self.desired_speed = None
        self.last_cmd_time = None

    def on_tick(self, data):
        if (self.desired_speed is None or
            self.last_cmd_time is None or
            self.time - self.last_cmd_time >= self.control_timeout):
            self.send_speed_cmd(0, 0)
        else:
            self.send_speed_cmd(*self.desired_speed)

    def send_speed_cmd(self, forward, rotation):
        # Send the speed command.
        delta = self.wheel_base * rotation / 2
        left, right = int(round((forward - delta) * 1000)), int(round((forward + delta) * 1000))
        validity = int(round(self.max_command_validity * 1000))
        cmd = struct.pack('>Bhhh', ord('V'), right, left, validity)
        checksum = -sum(cmd) & 0xFF
        cmd += bytes([checksum])
        self.publish('cmd', cmd)

        # Immediately ask for the current state.
        req = ord('?')
        checksum = -req & 255
        cmd = bytes([req, checksum])
        self.publish('cmd', cmd)


    def on_desired_speed(self, data):
        forward, rotation = data
        self.desired_speed = forward / 1000, math.radians(rotation / 100)
        self.last_cmd_time = self.time

    def on_info(self, msg):
        checksum = sum(msg) & 0xFF
        if checksum != 0:
            print('Incorrect checksum: {}'.format(checksum))
            return

        if msg[0] == ord('!'):
            (x, y, phi, emergency_stop) = struct.unpack('>qqq?', msg[1:-1])
            self.publish('pose2d',
                         [round(x * 1e-3), round(y * 1e-3),
                             round(math.degrees(phi * 1e-4))])
            emergency_stop = bool(emergency_stop)
            if emergency_stop != self.emergency_stop_status:
                self.publish('emergency_stop', emergency_stop)
                self.emergency_stop_status = emergency_stop
        elif msg[0] == ord('E') and msg[:-1] != b'E\nOK V\n':
            print('Communication error: {}'.format(msg[:-1]))
            self.publish('stdout', msg[:-1].decode('ascii'))


class Demo(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')

    def on_pose2d(self, data):
        x, y, phi = data
        x, y = x / 1000, y / 1000
        # mm/s
        velocity = 0 if math.hypot(x, y) >= 1.0 else 100
        self.publish('desired_speed', [velocity, 0])

