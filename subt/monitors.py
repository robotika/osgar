
from datetime import timedelta

class TimeoutReached(Exception):
    pass


class Timeout:
    def __init__(self, robot, timeout):
        self.uuid = robot.random.getrandbits(64)
        self.robot = robot
        self.timeout = timeout
        self.was_timeout = False

    def update(self):
        self.timeout -= self.robot.time - self.time
        self.time = self.robot.time
        if self.timeout < timedelta():
            raise TimeoutReached(self.uuid)

    def __enter__(self):
        self.time = self.robot.time
        self.handle = self.robot.register(self.update)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.handle)
        if exc_val is not None and isinstance(exc_val, TimeoutReached):
            if exc_val.args[0] == self.uuid:
                self.was_timeout = True
                return True  # don't reraise

    def __bool__(self):
        return self.was_timeout


