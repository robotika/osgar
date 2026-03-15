"""
  Driver for a small tank Maria

  Text Communication Protocol
  Version 0 uses ASCII protocol between the host computer (JetsonNano) and Arduino motor controller.
  There is no speed control implemented on Arduino at the moment, so raw PWM is used:
     "<PWM left> <PWM right> <desired LED status>\n"
  The value for PWM is in range -255..255
  LED status: 0=OFF, 1=green, 2=red.

  The board regularly responds (20Hz?) with status line:
      "<timestamp> <encoder left> <encoder right> <buttons mask>\n"
  The timestamp is 32bit unsigned int in milliseconds.
  The encoders values are 32bit signed int and accumulative.
  The buttons mask is currently not used (first remote tests are with Emergency STOP pressed).

  The plan is to abandon this text protocol and use binary structures inspired by ATmega8board.
"""

import math

from osgar.node import Node
from osgar.bus import BusShutdownException


ENC_SCALE = 0.00052
WHEEL_DISTANCE = 0.4  # 50cm long, 40cm wide
UPDATE_RATE = 20.0  # Hz
MIN_PWM = 10
MAX_PWM = 60

GREEN_LED = 1
RED_LED = 2


class SpeedControl:
    def __init__(self):
        self.min_pwm = MIN_PWM
        self.max_pwm = MAX_PWM
        self.step_pwm = 2
        self.tolerance = 5  # do not change PWM output if it is in +/- tolerance

        self.desired_ticks = 0
        self.last_pwm = 0

    def set_desired_ticks(self, ticks):
        """desired encoder ticks per fixed period"""
        self.desired_ticks = ticks

    def update(self, encoder_ticks):
        """Regular status of encoders ticks, return new PWM"""
        pwm = self.last_pwm
        if self.desired_ticks == 0:
            pwm = 0
        elif self.desired_ticks > 0:
            if encoder_ticks < -self.tolerance:
                # change of polarity, transit via 0
                pwm = 0
            elif abs(encoder_ticks - self.desired_ticks) > self.tolerance:
                if encoder_ticks > self.desired_ticks:
                    pwm = max(self.last_pwm - self.step_pwm, self.min_pwm)
                else:
                    pwm = max(self.last_pwm + self.step_pwm, self.min_pwm)
        else:  # self.desired_ticks < 0
            if encoder_ticks > self.tolerance:
                pwm = 0
            elif abs(encoder_ticks - self.desired_ticks) > self.tolerance:
                if abs(encoder_ticks) > abs(self.desired_ticks):
                    pwm = -max(-self.last_pwm - self.step_pwm, self.min_pwm)
                else:
                    pwm = -max(-self.last_pwm + self.step_pwm, self.min_pwm)
        pwm = max(-self.max_pwm, min(self.max_pwm, pwm))  # clip limits
        self.last_pwm = pwm
        return pwm


class RobotMaria(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose2d', 'emergency_stop', 'encoders', 'raw')

        # commands
        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0
        self.sc_left = SpeedControl()
        self.sc_right = SpeedControl()

        # status
        self.emergency_stop = None  # uknown state
        self.pose2d = (0.0, 0.0, 0.0)  # x, y in meters, heading in radians (not corrected to 2PI)
        self.buttons = None

        self.verbose = False  # should be in Node
        self.buf = b''
        self.encoder = None
        self.led = GREEN_LED
        self.milisec = None

    def send_pose2d(self):
        x, y, heading = self.pose2d
        self.publish('pose2d', [round(x*1000), round(y*1000),
                                round(math.degrees(heading)*100)])


    def compute_pose2d(self, left, right):
        """Update internal pose2d"""
        if left is None or right is None:
            return False, None, None
        x, y, heading = self.pose2d

        metricL = ENC_SCALE * left
        metricR = ENC_SCALE * right

        dist = (metricL + metricR)/2.0
        angle = (metricR - metricL)/WHEEL_DISTANCE

        # advance robot by given distance and angle
        if abs(angle) < 0.0000001:  # EPS
            # Straight movement - a special case
            x += dist * math.cos(heading)
            y += dist * math.sin(heading)
            #Not needed: heading += angle
        else:
            # Arc
            r = dist / angle
            x += -r * math.sin(heading) + r * math.sin(heading + angle)
            y += +r * math.cos(heading) - r * math.cos(heading + angle)
            heading += angle  # intentionally not normalized
            #  Explanation:
            #    Pose2D from odometry is one exception where you can collect
            #  history, i.e. that robot turned twice 360 degrees for example.
            #  This is different to compass where you have only current heading.
        return True, (x, y, heading), (dist, angle)

    def update_pwm0(self, encoders_left, encoders_right):
        STEP = int(min(255, abs(100 * self.desired_speed)))  # test PWM
        if STEP == 0:
            STEP = 200  # hardcoded for rotaion in place
        if self.desired_speed > 0:
            left, right = STEP, STEP
        elif self.desired_speed < 0:
            left, right = -STEP, -STEP
        elif self.desired_angular_speed > 0:
            left, right = -STEP, STEP
        elif self.desired_angular_speed < 0:
            left, right = STEP, -STEP
        else:
            left, right = 0, 0  # STOP

        self.publish('raw', b'%d %d %d\n' % (left, right, self.led))

    def update_pwm(self, encoders_left, encoders_right):
        left = self.sc_left.update(encoders_left)
        right = self.sc_right.update(encoders_right)
        self.publish('raw', b'%d %d %d\n' % (left, right, self.led))

    def get_packet(self, buf):
        s = buf.split(b'\n')
        if len(s) < 2:
            return None, buf
        return s[0].strip(), b'\n'.join(s[1:])

    def slot_desired_speed(self, timestamp, data):
        self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)
        left = self.desired_speed - self.desired_angular_speed * WHEEL_DISTANCE/2
        right = self.desired_speed + self.desired_angular_speed * WHEEL_DISTANCE/2
        self.sc_left.set_desired_ticks(left/ENC_SCALE/UPDATE_RATE)
        self.sc_right.set_desired_ticks(right/ENC_SCALE/UPDATE_RATE)

    def slot_raw(self, timestamp, data):
        self.buf += data
        packet, self.buf = self.get_packet(self.buf)
        while packet is not None:
            try:
                d = [int(x) for x in packet.split()]
            except ValueError:
                print('Invalid packet', packet.split())
                packet, self.buf = self.get_packet(self.buf)
                continue

            if self.verbose:
                print(d)
            assert len(d) == 4, d
            prev = self.encoder
            self.milisec = d[0]
            self.encoder = d[1], d[2]
            if prev is not None:
                left, right = self.encoder[0] - prev[0], self.encoder[1] - prev[1]
                self.publish('encoders', [left, right])
                valid, pose2d, dist_angle = self.compute_pose2d(left, right)
                if valid:
                    self.pose2d = pose2d
                self.send_pose2d()
                self.update_pwm(left, right)

            packet, self.buf = self.get_packet(self.buf)

    def run(self):
        try:
            while True:
                self.time, channel, data = self.listen()

                if channel == 'raw':
                    self.slot_raw(self.time, data)
                elif channel == 'desired_speed':
                    self.slot_desired_speed(self.time, data)
                else:
                    assert False, channel  # unsupported channel
        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
