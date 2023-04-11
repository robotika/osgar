"""
    A simple robotic simulator to become familiar with the Osgar.
"""
import math
import cv2
import numpy as np
import matplotlib.pyplot as plt
from osgar.node import Node


def create_map(map_data):
    base_space = np.ones((1100, 1100), dtype=np.uint8)*255
    base_space[50:-50, 50:-50] = np.zeros((1000, 1000), dtype=np.uint8)
    return base_space


class OsgarSimulator(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose2d')
        map_data = config.get("map_data")
        fig = plt.figure()
        self.ax = fig.add_subplot(111)
        self.base_map = create_map(map_data)
        self.robot_center = np.array([100, 100], dtype=np.uint8)
        wheel = np.array([  # front left wheel
            [[75, 125]],
            [[80, 125]],
            [[80, 105]],
            [[75, 105]]
            ], dtype=np.int32
        )
        self.robot_contour = [
            np.array([
                [[80, 120]],
                [[120, 120]],
                [[120, 80]],
                [[80, 80]]
            ], dtype=np.int32),
            wheel,
            wheel + np.array([45, 0]),
            wheel + np.array([0, -30]),
            wheel + np.array([45, -30])
            ]
        self.robot_arr = np.zeros(self.base_map.shape)
        self.speed = 0
        self.angular_speed = 0
        self.collision = False

    def on_desired_speed(self, data):
        speed, angular_speed = data
        self.speed = speed / 1000,
        self.angular_speed = math.radians(angular_speed / 100)

    def move_robot(self):
        current_robot_arr = cv2.drawContours(self.robot_arr.copy(), self.robot_contour, 0, color=(255), thickness=-1)

    def draw_map(self):
        map_im = cv2.drawContours(self.base_map.copy(), self.robot_contour, 0, color=(100), thickness=-1)
        map_im = cv2.drawContours(map_im, self.robot_contour[1:], -1, color=(180), thickness=-1)
        self.ax.imshow(map_im)
        plt.draw()
        self.ax.set_xlim(0, 1100)
        self.ax.set_ylim(0, 1100)
        plt.show()

    def update(self):
        super().update()
        pose2d = self.move_robot()
        self.draw_map()
        self.publish("pose2d", pose2d)
