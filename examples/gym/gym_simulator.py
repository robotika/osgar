"""
    A simple robotic simulator to become familiar with the Osgar.
"""

import math
import yaml
from argparse import Namespace
import numpy as np

import gym
from f110_gym.envs.base_classes import Integrator

from osgar.node import Node
from osgar.bus import BusShutdownException


class GymSimulator(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose2d', 'scan', 'sim_time')
        map_config = config['map_config']
        with open(map_config) as file:
            conf_dict = yaml.load(file, Loader=yaml.FullLoader)
        self.conf = Namespace(**conf_dict)
        self.speed = 0
        self.steer = 0

    def on_desired_speed(self, data):
        speed, angular_speed = data
        self.speed = speed / 1000
        angular_speed = math.radians(angular_speed / 100)
        if self.speed != 0:
            self.steer = math.atan(0.58*angular_speed/self.speed)  # 0.58 is robot length?
        else:
            self.steer = 0

    def on_move(self, data):
        speed, steering_angle_100deg = data
        self.speed = speed / 1000
        self.steer = math.radians(steering_angle_100deg / 100.0)

    def on_tick(self, data):
        # step simulation
        pass

    def send_data(self, obs):
        scan = (obs["scans"][0]*1000).astype(np.int16).tolist()
        self.publish("scan", scan)
        x = obs["poses_x"][0]
        y = obs["poses_y"][0]
        heading = obs["poses_theta"][0]
        self.publish('pose2d', [round(x * 1000), round(y * 1000), round(math.degrees(heading) * 100)])

    def run(self):
        try:
            racecar_env = gym.make('f110_gym:f110-v0', map=self.conf.map_path, num_agents=1, timestep = 0.02,
                                   integrator=Integrator.RK4)
            obs, step_reward, done, info = racecar_env.reset(np.array([[self.conf.sx, self.conf.sy, self.conf.stheta]]))
            sim_time = 0
            racecar_env.render()
            # loops when env not done
            while not done:
                self.update()
                obs, step_reward, done, info = racecar_env.step(np.array([[self.steer, self.speed]]))
                racecar_env.render(mode='human')
                self.send_data(obs)
                sim_time += step_reward
                self.publish('sim_time', sim_time)

            self.request_stop()
        except BusShutdownException:
            pass
