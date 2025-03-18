"""
    A simple robotic simulator to become familiar with the Osgar.
"""

import pygame
import math
import zmq

from osgar.lib.serialize import serialize, deserialize


class Zmq:
    def __init__(self):
        context = zmq.Context.instance()
        self.socket_pull = context.socket(zmq.PULL)
        # https://stackoverflow.com/questions/7538988/zeromq-how-to-prevent-infinite-wait
        self.socket_pull.RCVTIMEO = 200  # milliseconds
        self.socket_pull.LINGER = 100
        self.socket_pull.bind("tcp://*:5555")

        # Another context for PUSH
        context = zmq.Context.instance()
        self.socket_push = context.socket(zmq.PUSH)
        self.socket_push.setsockopt(zmq.LINGER, 100)  # milliseconds
        self.socket_push.bind("tcp://*:5556")

    def pull_msg(self):
        try:
            channel, raw = self.socket_pull.recv_multipart()
            data = deserialize(raw)
            # print(channel, message)
            return channel.decode('ascii'), data

        except zmq.ZMQError as e: # zmq.error.Again:
            # print(e)
            pass

    def push_msg(self, msg):
        try:
            channel, data = msg
            raw = serialize(data)
            self.socket_push.send_multipart([bytes(channel, 'ascii'), raw])
        except zmq.ZMQError as e:
            pass


class RobotSimulator:
    def __init__(self):
        self.width, self.height = 800, 600
        self.bg_color = (30, 30, 30)
        self.robot_color = (0, 255, 0)
        self.border_color = (200, 0, 0)
        self.robot_size = (40, 20)
        self.max_speed = 5

        pygame.init()
        self.display = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Osgar simulator")
        self.clock = pygame.time.Clock()

        self.robot_x, self.robot_y = self.width // 2, self.height // 2
        self.robot_angle = 0
        self.robot_speed = 0
        self.running = True
        self.osgar_running = False
        self.zmq_server = Zmq()

    def set_desired_steering(self, data):
        speed, steering = data
        self.robot_speed = speed/1000
        self.robot_angle = steering/100

    def send_pose(self, x, y, heading):
        self.zmq_server.push_msg(("pose2d", (x, y, heading*100)))

    def step(self):
        new_x = self.robot_x + self.robot_speed * math.cos(math.radians(self.robot_angle))
        new_y = self.robot_y - self.robot_speed * math.sin(math.radians(self.robot_angle))

        if 5 < new_x < self.width - self.robot_size[0] - 5 and 5 < new_y < self.height - self.robot_size[1] - 5:
            self.robot_x, self.robot_y = new_x, new_y
            self.draw_robot()
        else:
            print("Collision!")
            self.robot_speed = 0

    def draw_robot(self):
        self.display.fill(self.bg_color)
        pygame.draw.rect(self.display, self.border_color, (0, 0, self.width, self.height), 5)

        robot_rect = pygame.Rect(0, 0, *self.robot_size)
        robot_rect.center = (self.robot_x, self.robot_y)
        rotated_robot = pygame.transform.rotate(pygame.Surface(self.robot_size), self.robot_angle)
        rotated_robot.fill(self.robot_color)
        self.display.blit(rotated_robot, rotated_robot.get_rect(center=robot_rect.center))
        # pygame.display.flip()
        pygame.display.update()

    def run(self):
        while self.running:
            msg = self.zmq_server.pull_msg()
            if msg:
                print(msg)
                channel, data = msg
                if channel == "desired_steering":
                    self.set_desired_steering(data)

                if channel == "tick":
                    self.send_pose(self.robot_x*10, self.robot_y*10, self.robot_angle)  # TODO scale

            self.step()
            self.clock.tick(10)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pygame.quit()


if __name__ == "__main__":
    with RobotSimulator() as s:
        s.run()
