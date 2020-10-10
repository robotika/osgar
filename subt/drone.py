"""
  OSGAR Drone for control of flight height
"""
# copy of drone_height.py
import math

from osgar.node import Node

HEIGHT = 2.0
MAX_ANGULAR = 0.7
MAX_VERTICAL = 0.7
PID_P = 1.0  # 0.5
PID_I = 0.0  # 0.5


class Drone(Node):
    #class that listens all topics needed for controlling drone height
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_speed_3d")

        self.lastScanDown = 0.0
        self.lastScanUp = 0.0
        self.started = False

#        self.subscriberTwist = rospy.Subscriber("/cmd_vel", Twist, self.twistCallback, queue_size=15)
#        self.subscriberScanDown = rospy.Subscriber("/scan_down", LaserScan, self.scanDownCallback, queue_size=15)
#        self.subscriberScanDown = rospy.Subscriber("/scan_up", LaserScan, self.scanUpCallback, queue_size=15)
#        self.publisherTwist = rospy.Publisher("/cmd_vel_drone", Twist, queue_size=50)

        self.accum = 0.0
        self.debug_arr = []
        self.verbose = False

    def on_bottom_scan(self, scan):
            self.lastScanDown = scan[0]

    def on_top_scan(self, scan):
        self.lastScanUp = scan[0]

    def on_desired_speed(self, data):
        linear = [data[0]/1000, 0.0, 0.0]
        angular = [0.0, 0.0, math.radians(data[1]/100.0)]

        if self.verbose:
            self.debug_arr.append((self.time, self.lastScanDown, self.lastScanUp))

        if data != [0, 0]:
            self.started = True  # TODO this logic has to be also in "rosmsg.py" and/or "ros_proxy_node.cc"

        if self.started:
            height = min(HEIGHT, (self.lastScanDown + self.lastScanUp) / 2)
            diff = height - self.lastScanDown
            self.accum += diff
            desiredVel = max(-MAX_VERTICAL, min(MAX_VERTICAL, PID_P * diff + PID_I * self.accum))
            linear[2] = desiredVel

            #this is because drone can't handle too aggressive rotation
            if angular[2] > MAX_ANGULAR:
                angular[2] = MAX_ANGULAR
            elif angular[2] < -MAX_ANGULAR:
                angular[2] = -MAX_ANGULAR

            self.publish('desired_speed_3d', [linear, angular])

    def update(self):
        channel = super().update()

        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

    def draw(self):
        import matplotlib.pyplot as plt

        t = [a[0].total_seconds() for a in self.debug_arr]
        height = [(-a[1], a[2]) for a in self.debug_arr]
        line = plt.plot(t, height, '-o', linewidth=2)

        plt.xlabel('time (s)')
        plt.axes().set_aspect('equal', 'datalim')
        plt.legend()
        plt.show()

# vim: expandtab sw=4 ts=4
