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
        self.vslam_fail_start = None

    def on_vslam_pose(self, data):
        super().on_vslam_pose(data)

        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return

        if math.isnan(data[0][0]): # VSLAM not tracking
            self.vslam_valid = False
            if (
                    self.tf['vslam']['trans_matrix'] is not None and
                    not self.inException and
                    not self.returning_to_base
            ):
                if self.vslam_fail_start is not None and self.sim_time - self.vslam_fail_start > timedelta(milliseconds=300):
                    self.vslam_fail_start = None
                    raise VSLAMLostException
                else:
                    self.vslam_fail_start = self.sim_time
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
        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return
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
                 # https://www.researchgate.net/figure/Minimum-overlap-of-circles-covering-an-area_fig2_256607366
                 # Minimum overlap of circles covering an area; r= 3.46 (=2* r * cos(30)); step 60 degrees, 6x
            for i in range(12):
                x_d,y_d = pol2cart(2*3.46, math.radians(30 + i * 60))
                heapq.heappush(self.volatile_queue, (3, [object_index, object_type, x+x_d, y+y_d]))


    def circular_pattern(self):
        start_time = self.sim_time
        while current_sweep_step < len(sweep_steps) and self.sim_time - start_time < timedelta(minutes=60):
            ex = None
            try:
                self.virtual_bumper = VirtualBumper(timedelta(seconds=5), 0.2) # radius of "stuck" area; a little more as the robot flexes
                with LidarCollisionMonitor(self, 1200): # some distance needed not to lose tracking when seeing only obstacle up front
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
                else:
                    print("VSLAM found after stepping back")
                continue
            except VSLAMFoundException as e:
                print("VSLAM found on way to base")
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
                self.go_straight(math.copysign(1, -speed), timeout=timedelta(seconds=10)) # go 1m in opposite direction
                # 2) turn back to original direction (in case the bump changed direction)
                self.turn(self.get_angle_diff(pos, speed), timeout=timedelta(seconds=15))
                # 3) go away on diagonal in opposite direction
                # 4) go forward fixed distance in original direction to avoid a boulder (distance = max size of boulder plus cos of diagonal)
                # 5) go back in on diagonal in opposite direction
                # 6) turn control over to original plan
                self.drive_around_rock(math.copysign(6,  speed)) # assume 6m the most needed
                self.inException = False
                #current_sweep_step += 1

    def stripe_to_obstacle_pattern(self):
        # sweep the hilly center in stripes (~10 mins), circle would include too many turns
        sweep_steps = []
        current_sweep_step = 0

        sweep_steps.append([[-23,-42], "goto", -self.default_effort_level]) # TODO: go to first point more aggressively, do not skip to next step
        for i in range(-42, 36, 3):
            sweep_steps.append([[35, i], "goto", self.default_effort_level])
            sweep_steps.append([[-23, i], "goto", -self.default_effort_level])
            sweep_steps.append([[-23, i+3], "side", self.default_effort_level])

        start_time = self.sim_time

        last_exception = self.sim_time
        while current_sweep_step < len(sweep_steps) and self.sim_time - start_time < timedelta(minutes=60):
            ex = None
            angle_diff = 0
            speed = 0
            pos = None
            try:
                self.virtual_bumper = VirtualBumper(timedelta(seconds=3), 0.2) # radius of "stuck" area; a little more as the robot flexes
                with LidarCollisionMonitor(self, 1500): # some distance needed not to lose tracking when seeing only obstacle up front
                    pos, op, speed = sweep_steps[current_sweep_step]
                    if op == "goto":
                        angle_diff = self.get_angle_diff(pos,speed)
                        if current_sweep_step > 0 and abs(angle_diff) > math.radians(90): # past target, move to next
                            print(self.sim_time, "Target angle diff is more than 90deg..., already past the target, go to next step")
                            current_sweep_step += 1
                            continue
                        if abs(angle_diff) > math.radians(10): # only turn if difference more than 10deg
                            ex = "turn"
                            self.turn(angle_diff, timeout=timedelta(seconds=15))
                        ex = "goto"
                        self.go_to_location(pos, speed)
                    elif op == "side":
                        self.move_sideways(3, view_direction=0) # look towards 0deg
                    current_sweep_step += 1
            except VSLAMLostException as e:
                print("VSLAM lost")
                self.inException = True
                if ex == "turn":
                    self.turn(-angle_diff, timeout=timedelta(seconds=15))
                else:
                    self.go_straight(math.copysign(3, -speed)) # go 3m in opposite direction
                self.inException = False
                self.returning_to_base = True # continuing plan based on odometry, VSLAM will hopefully catch up
                if not self.vslam_valid:
                    current_sweep_step += 1
            except VSLAMFoundException as e:
                self.returning_to_base = False
                print("VSLAM found")
            except LidarCollisionException as e:
                print(self.sim_time, "Lidar: stepping back from the obstacle...")
                self.inException = True
                self.go_straight(-2)
                self.inException = False
                if speed > 0: # if bump going forward, go backward to next leg
                    print(self.sim_time, "...and switching to backwards towards next leg")
                    current_sweep_step += 1
                else:
                    print(self.sim_time, "...and continuing backwards to existing target")

            except VirtualBumperException as e:
                if distance(self.xyz, pos) < 5:
                    print(self.sim_time, "Bumper: Close enough, continuing to next leg")
                    current_sweep_step += 1
                    continue
                print(self.sim_time, "Bumper: trying to reverse")
                self.inException = True
                self.go_straight(math.copysign(2, -speed))
                self.inException = False
                if self.sim_time - last_exception > timedelta(seconds=5):
                    last_exception = self.sim_time
                    print(self.sim_time, "- timeout, giving up on this leg, moving to next")
                    current_sweep_step += 1
                else:
                    print(self.sim_time, "- will try the same leg again")




    def run(self):

        # chord length=2*sqrt(h * (2* radius - h)) where h is the distance from the circle boundary
        # https://mathworld.wolfram.com/CircularSegment.html

        current_sweep_step = 0
        sweep_steps = []

        # go around the main crater in circles
        for rad in range(13,32,3): # create a little overlap not to miss anything
            for i in range(10):
                x,y = pol2cart(rad, 2 * math.pi - i * 2 * math.pi / 10)
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


        self.set_cam_angle(-0.1)
        self.set_light_intensity("0.5")

        def set_homebase_found(response):
            self.vslam_reset_at = self.sim_time

        try:
            self.wait_for_init()

            try:
                self.turn(math.radians(360), timeout=timedelta(seconds=40))
            except ChangeDriverException as e:
                self.send_speed_cmd(0.0, 0.0)
            finally:
                self.send_request('vslam_reset', set_homebase_found)

            while not self.true_pose:
                self.update()
            self.wait(timedelta(seconds=2)) # wait to receive first poses after true_pose before acting upon the current location

            # go to the nearest point on the circle?


            self.stripe_to_obstacle_pattern()
            # self.circular_pattern()

        except BusShutdownException:
            pass


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Space Robotics Challenge 2')
    args = parser.parse_args()

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
