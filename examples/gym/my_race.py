"""
    TODO
"""
import math

from osgar.node import Node
from osgar.explore import follow_wall_angle
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.lib.local_planner import LocalPlanner


class MyRace(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("move")
        self.bus = bus
        self.max_speed = config.get("max_speed", 0.8)
        use_local_planner = config.get("local_planner", False)
        self.scan = None
        self.verbose = False
        if use_local_planner:
            self.local_planner = LocalPlanner(
                direction_adherence=math.radians(45),
                scan_subsample=3,
                max_obstacle_distance=2,
                obstacle_influence=1.5
            )
        else:
            self.local_planner = None

    def send_speed(self, speed, desired_direction):
        # reverse direction
        return self.publish('move', [round(speed * 1000), round(math.degrees(desired_direction) * 100)])

    def go_safely(self, desired_direction, safety):
        # TODO stop if a obstacle is too close?
        # self.max_speed *= safety
        self.send_speed(self.max_speed, desired_direction)

    def on_pose2d(self, data):
        pass

    def on_scan(self, data):
        # data[-90:] = [0]*90
        self.scan = data
        safety = 1
        desired_direction = normalizeAnglePIPI(follow_wall_angle(self.scan, gap_size=1.5, wall_dist=1, right_wall=False))
        if desired_direction is not None:
            # print(self.time, desired_direction)
            if self.verbose:
                print("wall", self.time, desired_direction)
            if self.local_planner:
                self.local_planner.update(data)
                safety, desired_direction = self.local_planner.recommend(desired_direction)
                if self.verbose:
                    print("planner", desired_direction, "\n")
            # print("planer",safety, desired_direction, "\n")
            self.go_safely(desired_direction, safety)
