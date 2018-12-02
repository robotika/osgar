"""
  Test basic robot/John Deere driving functionality
"""
import math
from datetime import timedelta


class GoOneMeter:
    def __init__(self, config, bus):
        self.bus = bus
        self.start_pose = None
        self.traveled_dist = 0.0
        self.time = None
        self.verbose = False

    def send_speed_cmd(self, speed, angular_speed):
        return self.bus.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def update(self):
        packet = self.bus.listen()
        if packet is not None:
            if self.verbose:
                print('Go1m', packet)
            timestamp, channel, data = packet
            self.time = timestamp
            if channel == 'pose2d':
                x, y, heading = data
                pose = (x/1000.0, y/1000.0, math.radians(heading/100.0))
                if self.start_pose is None:
                    self.start_pose = pose
                self.traveled_dist = math.hypot(pose[0] - self.start_pose[0], pose[1] - self.start_pose[1])
                        
    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def run(self):
        print("Go One Meter!")
        self.send_speed_cmd(0.5, 0.0)
        while self.traveled_dist < 1.0:
            self.update()
        print("STOP")
        self.send_speed_cmd(0.0, 0.0)
        self.wait(timedelta(seconds=1))

    def start(self):
        pass

    def request_stop(self):
        self.bus.shutdown()

    def join(self):
        pass


if __name__ == "__main__":
    from osgar.launcher import launch

    launch(app=GoOneMeter, description='Go One Meter', prefix='go1m-')

# vim: expandtab sw=4 ts=4
