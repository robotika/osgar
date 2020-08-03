"""
  Space Robotics Challenge 2
"""
import math
import heapq

from datetime import timedelta
from functools import partial

from osgar.bus import BusShutdownException
from osgar.lib import quaternion
from osgar.lib.quaternion import euler_zyx

from osgar.lib.virtual_bumper import VirtualBumper

from moon.controller import (distance, SpaceRoboticsChallenge, VirtualBumperException, ChangeDriverException,  VSLAMLostException, VSLAMFoundException,
                             LidarCollisionException, LidarCollisionMonitor)

def pol2cart(rho, phi):
    x = rho * math.cos(phi)
    y = rho * math.sin(phi)
    return(x, y)


class SpaceRoboticsChallengeRound1(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.use_gimbal = False

        self.default_effort_level = 1000 # out of 1000; indicates how much effort compared to max should be used on driving

        self.volatile_queue = [] # each element is an array [index, name x, y]
        self.volatile_last_submit_time = None
        self.processing_plant_found = False
        self.vslam_reset_at = None
        self.vslam_valid = False
        self.returning_to_base = False

    def on_vslam_pose(self, data):
        super().on_vslam_pose(data)

        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return

        if math.isnan(data[0][0]): # VSLAM not tracking
            self.vslam_valid = False
            if self.tf['vslam']['trans_matrix'] is not None and not self.inException and not self.returning_to_base: # it was tracking so it is lost, go back and re-acquire lock
                raise VSLAMLostException
            return

        if not math.isnan(data[0][0]):
            self.vslam_valid = True
            if not self.inException and self.returning_to_base:
                raise VSLAMFoundException

        if self.vslam_reset_at is not None and self.sim_time - self.vslam_reset_at > timedelta(seconds=3) and not math.isnan(data[0][0]) and self.tf['vslam']['trans_matrix'] is None:
            # request origin and start tracking in correct coordinates as soon as first mapping lock occurs
            # TODO: another pose may arrive while this request is still being processed (not a big deal, just a ROS error message)
            self.send_request('request_origin', self.register_origin)

    def process_volatile_response(self, response, vol_index):
        print(self.sim_time, "app: Response for obj %d, response: %s, guesses in queue: %d" % (vol_index, response, len(self.volatile_queue)))
        if response == 'ok':
            # if ok, no need to attempt the remaining guesses
            i = 0
            while i < len(self.volatile_queue):
                if self.volatile_queue[i][1][0] == vol_index:
                    self.volatile_queue[i], self.volatile_queue[-1] = self.volatile_queue[-1], self.volatile_queue[i]
                    self.volatile_queue.pop()
                else:
                    i += 1
            heapq.heapify(self.volatile_queue)
        else:
            # do nothing, ie keep going around and try to match the view
            pass

    def on_sim_clock(self, t):
        super().on_sim_clock(t)
        if (
                self.sim_time is not None and
                len(self.volatile_queue) > 0 and
                (self.volatile_last_submit_time is None or self.sim_time - self.volatile_last_submit_time > timedelta(seconds=30))
        ):
            priority, vol = heapq.heappop(self.volatile_queue)
            self.volatile_last_submit_time = self.sim_time
            print(self.sim_time, "app: Reporting obj %d, priority %d, guesses in queue: %d" % (vol[0], priority, len(self.volatile_queue)))
            self.send_request('artf %s %f %f 0.0' % (vol[1], vol[2], vol[3]), partial(self.process_volatile_response, vol_index=vol[0]))

    def on_artf(self, data):
        artifact_type = data[0]
        if artifact_type == "homebase" and not self.processing_plant_found:
            self.processing_plant_found = True
            raise ChangeDriverException


    def on_object_reached(self, data):
        object_type, object_index = data

        if (
                self.tf['vslam']['trans_matrix'] is not None and
                self.tf['vslam']['timestamp'] is not None and
                self.sim_time - self.tf['vslam']['timestamp'] < timedelta(milliseconds=300)
        ):
            x,y,z = self.xyz
            print(self.sim_time, "app: Object %s[%d] reached at [%.1f,%.1f], queueing up 19 around" % (object_type, object_index, x, y))
            heapq.heappush(self.volatile_queue, (1, [object_index, object_type, x, y]))
            for i in range(6):
                x_d,y_d = pol2cart(3.46, math.radians(i * 60))
                heapq.heappush(self.volatile_queue, (2, [object_index, object_type, x+x_d, y+y_d]))
                 # -2, 0, 2 https://www.researchgate.net/figure/Minimum-overlap-of-circles-covering-an-area_fig2_256607366
                 # Minimum overlap of circles covering an area; r= 3.46 (=2* r * cos(30)); step 60 degrees, 6x
            for i in range(12):
                x_d,y_d = pol2cart(2*3.46, math.radians(30 + i * 60))
                heapq.heappush(self.volatile_queue, (3, [object_index, object_type, x+x_d, y+y_d]))
                # TODO: make it priority queue, center guesses top priority, first ring second priority



    def run(self):

        # chord length=2*sqrt(h * (2* radius - h)) where h is the distance from the circle boundary
        # https://mathworld.wolfram.com/CircularSegment.html

        current_sweep_step = 0
        sweep_steps = []

        # go around the main crater in circles
        for rad in range(13,32,3): # create a little overlap not to miss anything
            for i in range(10):
                x,y = pol2cart(rad, 2 * math.pi - i * 2 * math.pi / 10)
                y -= 4
                sweep_steps.append([[x,y], self.default_effort_level])

        # sweep the hilly center in stripes (~10 mins), circle would include too many turns
        for i in range(-13, 13, 3):
            sweep_steps.append([[-15,i], -self.default_effort_level])
            sweep_steps.append([[15,i], self.default_effort_level])


        sweep_steps.append([[-7,-52], -self.default_effort_level]) # go backwards to starting point of right stripe

        # sweep right strip
        for i in range(-65, -40, 3):
            sweep_steps.append([[-55,i], -self.default_effort_level])
            sweep_steps.append([[43,i], self.default_effort_level])


        self.set_cam_angle(-0.05)

        def set_homebase_found(response):
            self.vslam_reset_at = self.sim_time

        try:
            self.wait_for_init()
            start_time = self.sim_time
            self.set_light_intensity("0.3")

            try:
                self.turn(math.radians(360), timeout=timedelta(seconds=40))
            except ChangeDriverException as e:
                self.send_speed_cmd(0.0, 0.0)
            finally:
                self.send_request('vslam_reset', set_homebase_found)

            while not self.true_pose:
                self.update()
            self.wait(timedelta(seconds=2)) # wait to receive first poses after true_pose before acting upon the current location

            while current_sweep_step < len(sweep_steps) and self.sim_time - start_time < timedelta(minutes=60):
                ex = None
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=5), 0.2) # radius of "stuck" area; a little more as the robot flexes
                    with LidarCollisionMonitor(self, 2500): # some distance needed not to lose tracking when seeing only obstacle up front
                        pos, speed = sweep_steps[current_sweep_step]
                        angle_diff = self.get_angle_diff(pos,speed)
                        ex = "turn"
                        self.turn(angle_diff, timeout=timedelta(seconds=15))
                        ex = "move"
                        self.go_to_location(pos, speed)
                        current_sweep_step += 1
                        continue
                except VSLAMLostException as e:
                    print("VSLAM lost")
                    self.inException = True
                    if ex == "turn":
                        self.turn(-angle_diff, timeout=timedelta(seconds=15))
                    else:
                        self.go_straight(math.copysign(3, -speed), timeout=timedelta(seconds=10)) # go 3m in opposite direction
                    self.inException = False

                    if not self.vslam_valid: # if vslam still not valid after backtracking, go towards center
                        self.returning_to_base = True
                        sweep_steps.insert(current_sweep_step, [[0,0], self.default_effort_level])
                    continue
                except VSLAMFoundException as e:
                    print("VSLAM found")
                    self.returning_to_base = False
                    current_sweep_step += 1
                    continue
                except LidarCollisionException as e: #TODO: long follow of obstacle causes loss, go along under steeper angle
                    print(self.sim_time, "Lidar")
                    if distance(self.xyz, pos) < 5:
                        current_sweep_step += 1
                        continue
                    self.inException = True
                    if False and speed < 0: # going backwards
                        self.drive_around_rock(-6)
                    else:
                        self.lidar_drive_around(direction=-1)
                    self.inException = False
                    # continue to previous target from a position next to the rock
                    #current_sweep_step += 1
                except VirtualBumperException as e:
                    self.send_speed_cmd(0.0, 0.0)
                    print(self.sim_time, "Bumper")
                    if distance(self.xyz, pos) < 5:
                        current_sweep_step += 1
                        continue
                    self.inException = True
                    # evasion pattern:
                    # 1) pull back 0.5m so we can maneuver;
                    self.go_straight(math.copysign(0.5, -speed), timeout=timedelta(seconds=10)) # go 2m in opposite direction
                    # 2) turn back to original direction (in case the bump changed direction)
                    self.turn(self.get_angle_diff(pos, speed), timeout=timedelta(seconds=15))
                    # 3) go away on diagonal in opposite direction
                    # 4) go forward fixed distance in original direction to avoid a boulder (distance = max size of boulder plus cos of diagonal)
                    # 5) go back in on diagonal in opposite direction
                    # 6) turn control over to original plan
                    self.drive_around_rock(math.copysign(6,  speed)) # assume 6m the most needed
                    self.inException = False
                    #current_sweep_step += 1

        except BusShutdownException:
            pass


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Space Robotics Challenge 2')
    args = parser.parse_args()

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
