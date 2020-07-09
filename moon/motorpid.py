"""
  Motor PID controller
"""

class MotorPID:
    def __init__(self, p=1, i=0, d=0):
        self.desired_speed = 0.0
        self.param_p = p

    def set_desired_speed(self, speed):
        self.desired_speed = speed

    def update(self, current_speed):
        return self.param_p * (self.desired_speed - current_speed)

# vim: expandtab sw=4 ts=4
