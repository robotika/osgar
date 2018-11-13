"""
  Driver for robot Eduro
"""


import ctypes
import serial
import struct
import math
from threading import Thread

from osgar.logger import LogWriter, LogReader
from osgar.bus import BusShutdownException
from .canserial import CAN_packet

CAN_ID_SYNC = 0x80
CAN_ID_ENCODERS_LEFT = 0x181
CAN_ID_ENCODERS_RIGHT = 0x182
CAM_SYSTEM_STATUS = 0x8A  # including emergency STOP button
CAN_ID_BUTTONS = 0x28A

UPDATE_TIME_FREQUENCY = 20.0  # Hz
WHEEL_DIAMETER_LEFT = 427.0 / 445.0 * 0.26/4.0 + 0.00015
WHEEL_DIAMETER_RIGHT = 427.0 / 445.0 * 0.26/4.0 - 0.00015
WHEEL_DISTANCE = 0.315


def sint32_diff(a, b):
    return ctypes.c_int32(a - b).value


class Eduro(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus

        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0
        self.maxAcc = config.get('max_acc', 0.5)
        self._rampLastLeft, self._rampLastRight = 0.0, 0.0

        self.prev_enc_raw = {}
        self.dist_left_diff = 0
        self.dist_right_diff = 0


        self.emergency_stop = None  # uknown state
        self.pose = (0.0, 0.0, 0.0)  # x, y in meters, heading in radians (not corrected to 2PI)
        self.buttons = None  # unknown

    def update_encoders(self, msg_id, data):
        assert len(data) == 4, data
        arr = struct.unpack('<i', data)
        if msg_id in self.prev_enc_raw:
            diff = sint32_diff(arr[0], self.prev_enc_raw[msg_id])

            if msg_id == CAN_ID_ENCODERS_LEFT:
                self.dist_left_diff = diff
            elif msg_id == CAN_ID_ENCODERS_RIGHT:
                self.dist_right_diff = diff
        self.prev_enc_raw[msg_id] = arr[0]

    def update_emergency_stop(self, data):
        assert len(data) == 8, len(data)
        self.emergency_stop = (data[:2] == bytes([0,0x10])) and (data [3:] == bytes([0,0,0,0,0]))

    def update_pose(self):
        x, y, heading = self.pose

        scaleR = WHEEL_DIAMETER_RIGHT * math.pi/0x10000
        scaleL = WHEEL_DIAMETER_LEFT * math.pi/0x10000

        metricL = scaleL * self.dist_left_diff
        metricR = scaleR * self.dist_right_diff

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
            heading += angle # not normalized
        self.pose = (x, y, heading)

    def send_leds(self, blue, red):
        self.bus.publish('can', CAN_packet(0x30A, [0xFF, 0xFF, blue, red]))

    def update_buttons(self, data):
        assert len(data) == 2, len(data)
        val = struct.unpack('<H', data)[0] & 0x300
        if self.buttons is None or val != self.buttons:
            self.buttons = val
            msg = { 'blue_selected': ((val & 0x0200) == 0),
                    'cable_in': ((val & 0x0100) != 0) }
            if msg['blue_selected']:
                self.send_leds(blue=1, red=0)
            else:
                self.send_leds(blue=0, red=1)
            self.bus.publish('buttons', msg)

    def send_speed(self):
        self.SpeedL = self.desired_speed - self.desired_angular_speed * WHEEL_DISTANCE / 2.0
        self.SpeedR = self.desired_speed + self.desired_angular_speed * WHEEL_DISTANCE / 2.0 

        if False: #any(motorId in self.modulesForRestart for motorId in [0x01, 0x02]):
            # There is a motor in reset => we must stop (even aggressively)
            left, right = 0, 0
        else:
            scaleR = 1000.0 / ( math.pi * WHEEL_DIAMETER_RIGHT)
            scaleL = 1000.0 / ( math.pi * WHEEL_DIAMETER_LEFT )
            tmpSpeedL = self.SpeedL
            tmpSpeedR = self.SpeedR

        if self.maxAcc:
            # use ramps
            maxSpeedStep = self.maxAcc / UPDATE_TIME_FREQUENCY;
            if math.fabs( tmpSpeedL - self._rampLastLeft ) > maxSpeedStep or \
                math.fabs( tmpSpeedR - self._rampLastRight ) > maxSpeedStep:
                frac = maxSpeedStep / max( math.fabs( tmpSpeedL - self._rampLastLeft ), math.fabs( tmpSpeedR - self._rampLastRight ) )
                tmpSpeedL = self._rampLastLeft + frac * ( tmpSpeedL - self._rampLastLeft )
                tmpSpeedR = self._rampLastRight + frac * ( tmpSpeedR - self._rampLastRight )
                self._rampLastLeft = tmpSpeedL
                self._rampLastRight = tmpSpeedR

        left = int(scaleL * tmpSpeedL)
        right = int(scaleR * tmpSpeedR)

        maxLim = 4000
        if left > maxLim:
            right = right*maxLim//left
            left = maxLim
        if left < -maxLim:
            right = -right*maxLim//left
            left = -maxLim
        if right > maxLim:
            left = left*maxLim//right
            right = maxLim
        if right < -maxLim:
            left = -left*maxLim//right
            right = -maxLim

        self.bus.publish('can', CAN_packet(0x201, [
            left&0xff, (left>>8)&0xff,
            right&0xff, (right>>8)&0xff]))

    def send_speed_ref(self):
        if self.desired_speed > 0:
            left, right = 1000, 1000
        else:
            left, right = 0, 0
        self.bus.publish('can', CAN_packet(0x201, [
            left&0xff, (left>>8)&0xff,
            right&0xff, (right>>8)&0xff]))

    def check_restarted_modules(self, module_id, status):
        if module_id in [1, 2]:  # motors
            if status != 5:
                print(module_id, status)
                self.prev_enc_raw = {}
                self.dist_left_diff = 0
                self.dist_right_diff = 0

    def process_packet(self, packet, verbose=False):
        if len(packet) >= 2:
            msg_id = ((packet[0]) << 3) | (((packet[1]) >> 5) & 0x1f)
#            print(hex(msg_id), packet[2:])
            if msg_id in [CAN_ID_ENCODERS_LEFT, CAN_ID_ENCODERS_RIGHT]:
                self.update_encoders(msg_id, packet[2:])
            elif msg_id == CAM_SYSTEM_STATUS:
                self.update_emergency_stop(packet[2:])
                self.bus.publish('emergency_stop', self.emergency_stop)
                if self.emergency_stop:
                    print("Eduro - emergency stop")
                    self.bus.publish('can', CAN_packet(0, [0x80, 0]))  # sendPreOperationMode
            elif msg_id == CAN_ID_BUTTONS:
                self.update_buttons(packet[2:])
            elif msg_id == CAN_ID_SYNC:
                # make sure diff is reported _before_ update_pose() which reset counters!
                self.bus.publish('encoders', [self.dist_left_diff, self.dist_right_diff])
                self.update_pose()
                x, y, heading = self.pose
                self.bus.publish('pose2d', [round(x*1000), round(y*1000), round(math.degrees(heading)*100)])
                self.send_speed()
            elif msg_id & 0xFF0 == 0x700:  # heart beat message
                assert len(packet) == 3, len(packet)
                self.check_restarted_modules(msg_id & 0xF, packet[2])

    def process_gen(self, data, verbose=False):
        self.process_packet(data)
        yield None

    def run(self):
        try:
            while True:
                dt, channel, data = self.bus.listen()
                if channel == 'can':
                    if len(data) > 0:
                        for status in self.process_gen(data):
                            if status is not None:
                                self.bus.publish('can', status)  # TODO
                elif channel == 'desired_speed':
                    self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)
                else:
                    assert False, channel  # unsupported channel
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
