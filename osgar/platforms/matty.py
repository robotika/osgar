"""
  Matty - 4-wheel drive articulated robot with passive joint
"""
# https://robotika.vosrk.cz/robots/matty/
# https://robotika.cz/robots/matty-twins/

import math

from osgar.node import Node


WHEEL_DISTANCE = 0.645  # meters left and right rear wheel


class Matty(Node):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('can', 'emergency_stop', 'pose2d')
        self.max_speed = config.get('max_speed', 0.5)
        self.max_steering_deg = config.get('max_steering_deg', 45.0)
        self.last_steering = None
        self.last_speed = None
        self.last_emergency_stop = None
        self.last_vehicle_mode = None
        self.last_error_status = None
        self.last_bumpers = None
        self.last_left_speed = None
        self.last_right_speed = None
        self.pose = 0, 0, 0
        self.pose_counter = 0
        self.counters = {}

        self.desired_speed = 0.0  # m/s
        self.desired_steering_angle_deg = 0.0  # degrees
        self.debug_arr = []
        self.verbose = False

    def publish_pose2d(self, left, right):
        dt = 0.04  # 25Hz

        x, y, heading = self.pose

        metricL = left * dt
        metricR = right * dt

        dist = (metricL + metricR)/2.0
        angle = (metricR - metricL)/WHEEL_DISTANCE

        # advance robot by given distance and angle
        if abs(angle) < 0.0000001:  # EPS
            # Straight movement - a special case
            x += dist * math.cos(heading)
            y += dist * math.sin(heading)
            # Not needed: heading += angle
        else:
            # Arc
            r = dist / angle
            x += -r * math.sin(heading) + r * math.sin(heading + angle)
            y += +r * math.cos(heading) - r * math.cos(heading + angle)
            heading += angle  # not normalized
        self.pose = (x, y, heading)
        self.publish('pose2d', [round(x*1000), round(y*1000), round(math.degrees(heading)*100)])

    def on_esp_data(self, data):
        pass

    def on_desired_steering(self, data):
        """
        Store desired speed and steering angle.
        Input is the pair of speed in millimeters per second and steering angle in hundredth of degree
        """
        speed_mm_per_sec, steering_deg_hundredth = data
        if speed_mm_per_sec > 0:
            self.desired_gear = Gear.DRIVE
        elif speed_mm_per_sec < 0:
            self.desired_gear = Gear.REVERSE
        else:
            pass  # for zero leave it as it is now
        self.desired_speed = speed_mm_per_sec/1000  # m/s
        if self.max_speed is not None:
            self.desired_speed = min(self.max_speed, max(-self.max_speed, self.desired_speed))
        self.desired_steering_angle_deg = steering_deg_hundredth/100  # degrees
        if self.max_steering_deg is not None:
            self.desired_steering_angle_deg = min(self.max_steering_deg,
                                                  max(-self.max_steering_deg, self.desired_steering_angle_deg))
