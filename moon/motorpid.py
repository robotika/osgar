"""
  Motor PID controller
"""

class MotorPID:
    def __init__(self, p=1, i=0, d=0):
        self.desired_speed = 0.0
        self.current_speed = 0.0
        self.param_p = p

    def set_desired_speed(self, speed):
        self.desired_speed = speed

    def update(self, current_speed):
        self.current_speed = current_speed

    def get_effort(self):
        err = self.desired_speed - self.current_speed
        return self.param_p * err

# vim: expandtab sw=4 ts=4
