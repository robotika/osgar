"""
  Boat Marina 2.0 - experiments with osgar.logger and robot container
"""

import argparse
import sys
import time
import math
from datetime import timedelta

from osgar.logger import LogWriter, LogReader
from osgar.lib.config import load as config_load
from osgar.robot import Robot

from osgar.bus import BusHandler


TIMEOUT = timedelta(seconds=10)  # configurable via --duration parameter


################ COPY & PASTE ###################
from osgar.lib.route import Convertor, Route
from osgar.lib.line import distance


DESTINATION_RADIUS = 2.0
ANGLE_FRAC = 0.7  #0.5  #0.1  # average fraction for new measurement
MIN_GPS_REF_STEP = 1.0


def angleTo( f, t ):
    if math.fabs(f[0]-t[0]) < 0.0001 and math.fabs(f[1]-t[1]) < 0.0001:
        return 0
    return math.atan2( t[1]-f[1], t[0]-f[0] ) 


def normalizeAnglePIPI( angle ):
    while angle < -math.pi:
        angle += 2*math.pi
    while angle > math.pi:
        angle -= 2*math.pi
    return angle 


class GPSData:
    def __init__(self):
        self.lat = None
        self.lon = None
        self.timestamp = 0

    def set_milisec(self, data):
        if data is None or data[0] is None:
            self.lat = None
            self.lon = None
        else:
            self.lat = data[1]/3600000
            self.lon = data[0]/3600000


    def has_fix(self):
        return self.lat is not None


def navigate(boat, waypoints):
    "code taken from Magelan/RoboOrienteering"
    boat.update()
    conv = Convertor(refPoint = waypoints[0])

    # make sure GPS is captured
    while boat.gps is None or boat.gps.lat is None:
        print("NO GPS!")
        boat.update()

    # find next waypoint for interrupted journeyes
    route = Route(waypoints, conv=conv)
    indexEx = route.findNearestEx((boat.gps.lon, boat.gps.lat))
    print("NEAREST", indexEx)
    index = indexEx[2]
    index = 1
    if index < 0:
        index = -index

    prev_pos = None
    sumX, sumY = 0, 0
    gpsCompassAngle = None
    for goal in waypoints[index:]:        
        while True:
            if boat.gps is not None and boat.gps.has_fix():
                dist = distance(conv.geo2planar((boat.gps.lon, boat.gps.lat)), conv.geo2planar(goal))
                if dist < DESTINATION_RADIUS:
                    print("DESTINATION REACHED")
                    boat.set_desired_speed(0.0, 0.0)
                    boat.wait(5.0)
                    break            
                gpsAngle = angleTo(conv.geo2planar((boat.gps.lon, boat.gps.lat)), conv.geo2planar(goal))
                if prev_pos is None:
                    prev_pos = conv.geo2planar((boat.gps.lon, boat.gps.lat))
                x, y = conv.geo2planar((boat.gps.lon, boat.gps.lat))
                dx, dy = x - prev_pos[0], y - prev_pos[1]
                if math.hypot(dx, dy) > MIN_GPS_REF_STEP:
                    sumX = ANGLE_FRAC * dx + (1.0 - ANGLE_FRAC) * sumX
                    sumY = ANGLE_FRAC * dy + (1.0 - ANGLE_FRAC) * sumY
                    print("DX\t{}\t{}\t{}".format(boat.gps.timestamp, dx, dy))
                    if math.hypot(sumX, sumY) > 0.001:
                        gpsCompassAngle = math.atan2(sumY, sumX)
                        print("GPS angle = ", math.degrees(gpsCompassAngle))
                    else:
                        gpsCompassAngle = None
                    prev_pos = (x,y)
                    
            else:
                gpsAngle = None

            if boat.heading() is None or gpsAngle is None:
                if boat.heading() is None:
                    print("NO COMPASS!", boat.time)
                else:
                    print("NO COMPASS! (gps fix)", boat.time)
                boat.set_desired_speed(0, 0)
            else:
                compassAngle = boat.heading()
#                compassAngle = gpsCompassAngle
                if compassAngle is not None:
                    angular_speed = boat.angular_speed()
                    if angular_speed is None:
                        angular_speed = 0.0  # rad/sec
                    print("angular_speed (deg)", math.degrees(angular_speed))
#                    angular_speed = 0.0 # hack for now
#                    angular_speed = -angular_speed
#                    angular_speed = -angular_speed*0.5
#                    angular_speed = -angular_speed*2
                    angular_speed *= 2  # former version did not correct gyro direction

                    diff = math.degrees(normalizeAnglePIPI(gpsAngle-compassAngle-angular_speed))
                    print("ANGLES DIFF =", diff, dist)  #, (gpsAngle, compassAngle)
                    if abs(diff) < 20:
#                        boat.set_desired_speed(DESIRED_SPEED, 0.0)
                        boat.set_desired_speed_raw(600, 1000)
                    elif diff > 0:
                        print("TURN LEFT")
#                        boat.set_desired_speed(DESIRED_SPEED, math.radians(90))
#                        boat.set_desired_speed_raw(1000, 900)
#                        boat.set_desired_speed_raw(760, 931)
                        boat.set_desired_speed_raw(891, 792)
                    else:
                        print("TURN RIGHT")
                        assert diff < 0, diff
#                        boat.set_desired_speed(DESIRED_SPEED, math.radians(-90))
#                        boat.set_desired_speed_raw(1000, 1100)
                        boat.set_desired_speed_raw(760, 1200)
                else:
                    print("MISSING COMPASS ANGLE!")
                    boat.set_desired_speed(DESIRED_SPEED, 0)  # hack - move to get GPS estimage (!)
            boat.update()
            if boat.time > TIMEOUT:  # TODO raise timeout??
                break
        print("DESTINATION WITHIN %0.2f" % distance(conv.geo2planar((boat.gps.lon, boat.gps.lat)), conv.geo2planar(goal)))
        if boat.time > TIMEOUT:
            break

#################################################


class BoatMarina2:

    def __init__(self, config, bus):
        self.bus = bus
        self.time = None

        self.gps = GPSData()
        self._heading = None
        self._angular_speed = None
        self.channel_move = 1000
        self.channel_turn = 1000

    def heading(self):
        return self._heading  # TODO refactor

    def angular_speed(self):
        return self._angular_speed

    def set_desired_speed(self, speed, angular_speed):
        # legacy function
        assert speed == 0, speed
        assert angular_speed == 0, angular_speed
        self.set_desired_speed_raw(1000, 1000)

    def set_desired_speed_raw(self, speed, angular_speed):
        self.channel_move = speed
        self.channel_turn = angular_speed

    def update(self):
        packet = self.bus.listen()
        if packet is not None:
            timestamp, channel, data = packet
            self.time = timestamp
            if channel == 'position':
                assert len(data) == 2, data
                self.gps.set_milisec(data)
            elif channel == 'heading':
                if data is None:
                    self._heading = data
                else:
                    self._heading = math.radians(data/100)
                self.bus.publish('move', [self.channel_move, self.channel_turn])
            elif channel == 'angular_speed':
                self._angular_speed = math.radians(data/14.375)  # copy & paste from I2CLibraries

    def start(self):
        pass

    def play0(self):
        self.bus.publish('move', [999, 998])  # uniq values for test
        for i in range(10):
            time.sleep(1)  # TODO use self.time/wait()
            self.bus.publish('move', [600, 1000])
        self.bus.publish('move', [1000, 1000])
        time.sleep(1)  # TODO use self.time/wait() ... it has to pass through
               # TODO it should be confirmed from boat that it if already off

    def play(self):
        # Frymbruk - short
        waypoints = [(14.164983, 48.6585656667), (14.164965, 48.6586993), (14.1646324, 48.6583097), (14.164983, 48.6585656667)]
        # TODO move to config        
        navigate(self, waypoints)
        self.set_desired_speed_raw(1000, 1000)
        self.wait(timedelta(seconds=1))

    def request_stop(self):
        self.bus.shutdown()

    def join(self):
        pass

    def register(self, callback):
        self.monitors.append(callback)
        return callback

    def unregister(self, callback):
        assert callback in self.monitors
        self.monitors.remove(callback)

    def wait(self, dt):
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Boat Marina 2.0')
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', nargs='+', help='configuration file')
    parser_run.add_argument('--note', help='add description')
    parser_run.add_argument('--duration', help='run navigation for given number of seconds', type=int)

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    parser_replay.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser_replay.add_argument('--config', nargs='+', help='force alternative configuration file')
    parser_replay.add_argument('--duration', help='run navigation for given number of seconds', type=int)  # TODO parse from cmd line
    args = parser.parse_args()

    if args.duration is not None:
        TIMEOUT = timedelta(seconds=args.duration)

    if args.command == 'replay':
        from replay import replay
        args.module = 'app'
        game = replay(args, application=BoatMarina2)
        game.play()

    elif args.command == 'run':
        log = LogWriter(prefix='boat-', note=str(sys.argv))
        config = config_load(*args.config)
        log.write(0, bytes(str(config), 'ascii'))  # write configuration
        robot = Robot(config=config['robot'], logger=log, application=BoatMarina2)
        game = robot.modules['app']  # TODO nicer reference
        robot.start()
        game.play()
        robot.finish()
    else:
        assert False, args.command  # unsupported command

# vim: expandtab sw=4 ts=4
