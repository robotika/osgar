"""
  Simple IMU data parsing (VectorNav VN-100)
"""

from threading import Thread
import math

from osgar.bus import BusShutdownException
from osgar.lib.quaternion import euler_to_quaternion
from osgar.drivers.gps import checksum  # NMEA checksum used for $VNYMR message


def parse_line(line):
    """
    Parse $VNYMR message:
        Yaw  float  deg  Calculated attitude heading angle in degrees.
        Pitch  float  deg  Calculated attitude pitch angle in degrees.
        Roll  float  deg  Calculated attitude roll angle in degrees.
        MagX  float  Gauss  Compensated magnetometer measurement in x-axis.
        MagY  float  Gauss  Compensated magnetometer measurement in y-axis.
        MagZ  float  Gauss  Compensated magnetometer measurement in z-axis.
        AccelX  float  m/s^2 Compensated accelerometer measurement in x-axis.
        AccelY  float  m/s^2 Compensated accelerometer measurement in y-axis.
        AccelZ  float  m/s^2 Compensated accelerometer measurement in z-axis.
        GyroX  float  rad/s  Compensated angular rate in x-axis.
        GyroY  float  rad/s  Compensated angular rate in y-axis.
        GyroZ  float  rad/s  Compensated angular rate in z-axis.
    """
    assert line.startswith(b'$VNYMR'), line
    assert b'*' in line, line
    if checksum(line[1:-3]) != line[-2:]:
        print('Checksum error!', line, checksum(line[1:-3]))
        return None
    s = line.split(b'*')[0].split(b',')
    assert len(s) == 13, s
    arr = [float(x) for x in s[1:]]
    return [arr[:3], arr[3:6], arr[6:9], arr[9:]]


class IMU(Thread):
    def __init__(self, config, bus):
        bus.register('orientation', 'rotation', 'data')
        Thread.__init__(self)
        self.setDaemon(True)
        self.offset = config.get("offset", [0, 0, 0])  # rotation offset in 1/100th of degree
        self.bus = bus
        self.buf = b''
        self.verbose = False

    # Copy & Paste from gps.py - refactor!
    @staticmethod
    def split_buffer(data):
        start = data.find(b'$')
        if start < 0:
            return data, b''
        end = data[start:-2].find(b'*')
        if end < 0:
            return data, b''
        return data[start+end+3:], data[start:start+end+3]

    @staticmethod
    def process_packet(line):
        if line.startswith(b'$VNYMR'):
            result = parse_line(line)
            if result:
                return result
        return None

    def process_gen(self, data):
        self.buf, packet = self.split_buffer(self.buf + data)
        while len(packet) > 0:
            ret = self.process_packet(packet)
            if ret is not None:
                yield ret
            # now process only existing (remaining) buffer
            self.buf, packet = self.split_buffer(self.buf)  

    def run(self):
        try:
            while True:
                packet = self.bus.listen()
                dt, __, data = packet
                for out in self.process_gen(data):
                    assert out is not None
                    self.bus.publish('data', out)
                    # publish separately yaw, pitch and roll in 1/100th deg
                    yaw, pitch, roll = out[0]
                    # The  VN-100  uses a right-handed coordinate system. A positive yaw angle
                    # is defined as a positive righthanded rotation around the Z-axis. (pointing down)
                    yaw = 90.0 - yaw
                    if yaw < -180:
                        yaw += 360
                    if yaw > 180:
                        yaw -= 360
                    yaw, pitch, roll = [a + b/100 for a, b in zip([yaw, pitch, roll], self.offset)]  # correct for offset
                    quat = euler_to_quaternion(math.radians(yaw), math.radians(pitch), math.radians(roll))
                    angles = [int(round(x * 100)) for x in [yaw, pitch, roll]]
                    self.bus.publish('rotation', angles)
                    if self.verbose:
                        print(angles)
                    self.bus.publish('orientation', quat)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


if __name__ == "__main__":
    import io
    import sys
    import matplotlib.pyplot as plt
    
    arr = []
    for line in io.open(sys.argv[1]):
        angle, mag, acc, gyro = parse_line(line)
        arr.append(angle)

    plt.plot(arr, 'o-', linewidth=2)
    plt.show()

# vim: expandtab sw=4 ts=4
