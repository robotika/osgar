import logging

g_logger = logging.getLogger(__name__)


class TimeoutReached(Exception):
    pass


class TimeoutMonitor:
    def __init__(self, robot, timeout):
        self.robot = robot
        self.timeout = timeout
        self.fired = False

    def update(self):
        if (self.robot.time - self.start_time) > self.timeout:
            raise TimeoutReached(self)

    def __enter__(self):
        self.start_time = self.robot.time
        self.fired = False
        self.handle = self.robot.register(self.update)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.handle)
        if exc_val is not None and isinstance(exc_val, TimeoutReached):
            if exc_val.args[0] == self:
                self.fired = True
                g_logger.info("Timeout {self.timeout} reached.")
                return True  # don't reraise

    def __bool__(self):
        return self.fired


class PitchError(Exception):
    pass


class RollError(Exception):
    pass


class PitchMonitor:
    def __init__(self, robot, pitch_limit):
        self.robot = robot
        self.pitch_limit = pitch_limit
        self.fired = False

    def update(self):
        if self.pitch_limit is not None and self.robot.pitch is not None:
            if abs(self.robot.pitch) > self.pitch_limit:
                raise PitchError(self)

    def __enter__(self):
        self.fired = False
        self.handle = self.robot.register(self.update)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.handle)
        if exc_val is not None and isinstance(exc_val, PitchError):
            if exc_val.args[0] == self:
                self.fired = True
                g_logger.info("Pitch limit {math.degrees(self.pitch_limit)} reached.")
                return True  # don't reraise

    def __bool__(self):
        return self.fired


class RollMonitor:
    def __init__(self, robot, roll_limit):
        self.robot = robot
        self.roll_limit = roll_limit
        self.fired = False

    def update(self):
        if self.roll_limit is not None and self.robot.roll is not None:
            if abs(self.robot.roll) > self.roll_limit:
                raise RollError(self)

    def __enter__(self):
        self.fired = False
        self.handle = self.robot.register(self.update)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.handle)
        if exc_val is not None and isinstance(exc_val, RollError):
            if exc_val.args[0] == self:
                self.fired = True
                g_logger.info("Pitch limit {math.degrees(self.roll_limit)} reached.")
                return True  # don't reraise

    def __bool__(self):
        return self.fired
