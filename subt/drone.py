"""
  OSGAR Drone for control of flight height
"""
# copy of drone_height.py
import math

from osgar.node import Node

HEIGHT = 2.0
EMERGENCY_HEIGHT = 0.5
MAX_ANGULAR = 0.7
MAX_VERTICAL = 1
PID_P = 2.0  # 0.5


def altitude_from_pressure(p):
    # Model and parameters taken from Wikipedia:
    # https://en.wikipedia.org/wiki/Barometric_formula
    # I have not yet found corresponding atmospheric model and constants
    # in Ignition Gazebo.

    # Reference pressure. [Pa]
    Pb = 101325.00
    # Reference temperature. [K]
    Tb = 288.15
    # Gravitational acceleration. [m/s^2]
    g0 = 9.80665
    # Universal gas constant. [J/(mol*K)]
    R = 8.3144598
    # Molar mass of air. [kg/mol]
    M = 0.0289644
    # Height of reference level. [m]
    hb = 0

    # The pressure model is:
    #   p = Pb * exp(-g0 * M * (h - hb) / (R * Tb))
    # Therefore:
    h = (math.log(p) - math.log(Pb)) * (R * Tb) / (-g0 * M) + hb

    return h


class Drone(Node):
    #class that listens all topics needed for controlling drone height
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_speed_3d")

        self.count = 0 # count of bottom messages
        self.lastScanDown = 0.0
        self.lastScanUp = 0.0
        self.started = False

        self.debug_arr = []
        self.debug_arr2 = []
        self.verbose = False
        self.z = None  # last Z coordinate
        self.altitude = None  # based on air_pressure
        self.desired_z_speed = None  # i.e. keep safe distance from up and down obstacles

        # keep last command for independent update of XY and Z
        self.linear = [0.0, 0.0, 0.0]
        self.angular = [0.0, 0.0, 0.0]

    def send_speed_cmd(self):
        if self.started:
            H, E, A = 0, 0, 0
            desiredVel = None
            if self.desired_z_speed is None:
                height = min(HEIGHT, (self.lastScanDown + self.lastScanUp) / 2)
                diff = height - self.lastScanDown
                H = diff
                desiredVel = PID_P * diff
            else:
                if min(self.lastScanDown, self.lastScanUp) < EMERGENCY_HEIGHT:
                    print(self.time, 'Emergency limit', self.lastScanDown, self.lastScanUp)
                    # temporary switch to keep safe height from bottom/top
                    down = self.lastScanDown - EMERGENCY_HEIGHT
                    up = self.lastScanUp - EMERGENCY_HEIGHT
                    if up < 0 and down < 0:
                        diff = (up - down) / 2
                    elif up < 0:
                        diff = up
                    else:
                        diff = -down
                    E = diff
                    desiredVel = PID_P * diff
                else:
                    A = self.desired_z_speed
                    desiredVel = self.desired_z_speed

            desiredVel = max(-MAX_VERTICAL, min(MAX_VERTICAL, desiredVel))
            self.linear[2] = desiredVel

            #this is because drone can't handle too aggressive rotation
            if self.angular[2] > MAX_ANGULAR:
                self.angular[2] = MAX_ANGULAR
            elif self.angular[2] < -MAX_ANGULAR:
                self.angular[2] = -MAX_ANGULAR

            self.publish('desired_speed_3d', [self.linear, self.angular])
            if self.verbose:
                self.debug_arr2.append((self.count, H, E, A))

    def on_bottom_scan(self, scan):
        self.count += 1
        self.lastScanDown = scan[0]
        self.send_speed_cmd()

    def on_top_scan(self, scan):
        self.lastScanUp = scan[0]

    def on_desired_speed(self, data):
        self.linear = [data[0]/1000, 0.0, 0.0]
        self.angular = [0.0, 0.0, math.radians(data[1]/100.0)]

        if self.verbose:
            self.debug_arr.append((self.time, self.altitude, self.lastScanDown, self.lastScanUp, self.z))

        if data != [0, 0]:
            self.started = True

        self.send_speed_cmd()

    def on_desired_z_speed(self, z_speed):
        self.desired_z_speed = z_speed

    def on_pose3d(self, data):
        xyz, quat = data
        self.z = xyz[2]

    def on_air_pressure(self, data):
        self.altitude = altitude_from_pressure(data)

    def on_desired_altitude(self, data):
        pass  # self.desired_altitude already updated by Node

    def update(self):
        channel = super().update()

        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, "unknown input {channel}"


    def draw(self):
        import matplotlib.pyplot as plt

        #t = [a[0].total_seconds() for a in self.debug_arr]
        #height = [(a[1], a[1] - a[2], a[1] + a[3], a[4]) for a in self.debug_arr]
        #line_obj = plt.plot(t, height, '-o', linewidth=2)
        #plt.legend(iter(line_obj), ('altitude', 'alt - bottom', 'alt + top', 'pose3D.z'))
        #plt.xlabel('time (s)')
        x = [count/50.0 for count, H, E, A in self.debug_arr2]  # bottom lidar runnig at 50Hz
        a = [A for count, H, E, A in self.debug_arr2]
        e = [E for count, H, E, A in self.debug_arr2]
        h = [H for count, H, E, A in self.debug_arr2]
        plt.plot(x, a)
        plt.plot(x, e)
        plt.plot(x, h)
        plt.xlabel('sim time (s)')
        plt.legend()
        plt.show()

# vim: expandtab sw=4 ts=4
