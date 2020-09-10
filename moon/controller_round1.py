"""
  Space Robotics Challenge 2
"""
import math
import heapq
import numpy as np

from datetime import timedelta
from functools import partial

from osgar.bus import BusShutdownException
from osgar.lib import quaternion
from osgar.lib.quaternion import euler_zyx

from osgar.lib.virtual_bumper import VirtualBumper

from moon.controller import (pol2cart, ps, distance, SpaceRoboticsChallenge, VirtualBumperException, ChangeDriverException,  VSLAMLostException, VSLAMFoundException,
                             VSLAMEnabledException, VSLAMDisabledException, LidarCollisionException, LidarCollisionMonitor)

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
        self.vslam_is_enabled = False

    def get_extra_status(self):
        return ps("Volatile queue length: %d" % len(self.volatile_queue))

    def on_vslam_enabled(self, data):
        super().on_vslam_enabled(data)
        if self.vslam_is_enabled != data:
            self.vslam_is_enabled = data

            if not self.true_pose or self.sim_time is None or self.last_position is None or self.yaw is None:
                return

            if self.vslam_is_enabled:
                raise VSLAMEnabledException()
            else:
                raise VSLAMDisabledException()

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
                if self.vslam_fail_start is None:
                    self.vslam_fail_start = self.sim_time
                elif self.sim_time - self.vslam_fail_start > timedelta(milliseconds=300):
                    self.vslam_fail_start = None
                    raise VSLAMLostException()
            return

        self.vslam_fail_start = None

        if not math.isnan(data[0][0]):
            self.vslam_valid = True
            if not self.inException and self.returning_to_base:
                raise VSLAMFoundException()

        if self.vslam_reset_at is not None and self.processing_plant_found and self.sim_time - self.vslam_reset_at > timedelta(seconds=3) and not math.isnan(data[0][0]) and self.tf['vslam']['trans_matrix'] is None:
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

        # don't plan to handle anything but homebase
        assert artifact_type == "homebase", artifact_type

        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return

        if not self.processing_plant_found:
            self.processing_plant_found = True
            raise ChangeDriverException()


    def on_object_reached(self, data):
        object_type, object_index = data

        if (
                self.tf['vslam']['trans_matrix'] is not None and
                self.tf['vslam']['timestamp'] is not None and
                self.sim_time - self.tf['vslam']['timestamp'] < timedelta(milliseconds=300)
        ):
            x,y,z = self.xyz
            print(self.sim_time, "app: Object %s[%d] reached at [%.1f,%.1f], queueing up 19 around" % (object_type, object_index, x, y))
            heapq.heappush(self.volatile_queue, (self.sim_time.total_seconds(), [object_index, object_type, x, y]))
            for i in range(6):
                x_d,y_d = pol2cart(3.46, math.radians(i * 60))
                heapq.heappush(self.volatile_queue, (3600 + self.sim_time.total_seconds(), [object_index, object_type, x+x_d, y+y_d]))
                 # https://www.researchgate.net/figure/Minimum-overlap-of-circles-covering-an-area_fig2_256607366
                 # Minimum overlap of circles covering an area; r= 3.46 (=2* r * cos(30)); step 60 degrees, 6x
            for i in range(12):
                x_d,y_d = pol2cart(2*3.46 if i % 2 == 0 else 2*3, math.radians(i * 30))
                heapq.heappush(self.volatile_queue, (7200 + self.sim_time.total_seconds(), [object_index, object_type, x+x_d, y+y_d]))


    def circular_pattern(self):
        current_sweep_step = 0
        ARRIVAL_TOLERANCE = 5
        CIRCLE_SEGMENTS = 10
        sweep_steps = []
        #HOMEPOINT = self.xyz # where the drive started and we know the location well
        CENTERPOINT = [3,-3] # where to start the circle from
        HOMEPOINT = [-10,-10] # central area, good for regrouping
        print(self.sim_time, "Home coordinates: %.1f, %.1f" % (HOMEPOINT[0], HOMEPOINT[1]))


        #sweep_steps.append(["turn", [math.radians(360)]])
        sweep_steps.append(["goto", False, [HOMEPOINT, self.default_effort_level, False]])
        # sweep_steps.append(["turn", False, [math.radians(360)]]) # more likely to break vslam than help

        # go around the main crater in circles
        for rad in range(13,55,2): # create a little overlap not to miss anything
            for i in range(CIRCLE_SEGMENTS):
                x,y = pol2cart(rad, 2 * math.pi - i * 2 * math.pi / CIRCLE_SEGMENTS)
                sweep_steps.append(["goto", False, [[x + CENTERPOINT[0], y + CENTERPOINT[1]], self.default_effort_level, False]])

        # sweep the hilly center in stripes (~10 mins), circle would include too many turns
        # for i in range(-13, 13, 3):
        #    sweep_steps.append([[-15,i], -self.default_effort_level, False])
        #    sweep_steps.append([[15,i], self.default_effort_level, False])


        #sweep_steps.append([[-7,-52], -self.default_effort_level, False]) # go backwards to starting point of right stripe

        # sweep right strip
        #for i in range(-65, -40, 3):
        #    sweep_steps.append([[-55,i], -self.default_effort_level, False])
        #    sweep_steps.append([[43,i], self.default_effort_level, False])

        start_time = self.sim_time
        wait_for_mapping = False

        self.virtual_bumper = VirtualBumper(timedelta(seconds=5), 0.2) # radius of "stuck" area; a little more as the robot flexes
        while current_sweep_step < len(sweep_steps) and self.sim_time - start_time < timedelta(minutes=60):
            ex = None
            try:
                while wait_for_mapping:
                    self.wait(timedelta(seconds=1))
                with LidarCollisionMonitor(self, 1200): # some distance needed not to lose tracking when seeing only obstacle up front
                    op, self.inException, params = sweep_steps[current_sweep_step]
                    if op == "goto":
                        pos, speed, self.returning_to_base = params
                        print(self.sim_time, "Driving radius: %.1f" % distance(CENTERPOINT, pos))
                        self.go_to_location(pos, speed, full_turn=True, with_stop=False, tolerance=ARRIVAL_TOLERANCE, avoid_obstacles_close_to_destination=True)
                    elif op == "turn":
                        angle, self.returning_to_base = params
                        self.turn(angle, timeout=timedelta(seconds=20))
                    elif op == "straight": # TODO: if interrupted, it will repeat the whole distance when recovered
                        dist, self.returning_to_base = params
                        self.go_straight(dist)
                    elif op == "rock":
                        self.drive_around_rock(6) # assume 6m the most needed
                    elif op == "lidar":
                         self.lidar_drive_around()
                    else:
                        assert False, "Invalid command"
                    current_sweep_step += 1
            except VSLAMLostException as e:
                print("VSLAM lost")
                sweep_steps.insert(current_sweep_step, ["turn", False, [math.radians(360), True]])
                continue

                self.inException = True

                try:
                    self.go_straight(-3) # go 3m in opposite direction
                except VSLAMDisabledException as e:
                    print(self.sim_time, "VSLAM: mapping disabled, waiting")
                    self.send_speed_cmd(0.0, 0.0)
                    wait_for_mapping = True
                except VSLAMEnabledException as e:
                    print(self.sim_time, "VSLAM: mapping re-enabled")
                    wait_for_mapping = False
                except VSLAMFoundException as e:
                    print("VSLAM found on way to base")
                    self.returning_to_base = False
                    current_sweep_step += 1
                except BusShutdownException:
                    raise
                except:
                    pass

                self.inException = False
                if not self.vslam_valid: # if vslam still not valid after backtracking, go towards center
                    self.returning_to_base = True
                    # sweep_steps.insert(current_sweep_step, ["goto", [HOMEPOINT, self.default_effort_level, True]])
                    anglediff = self.get_angle_diff(HOMEPOINT)
                    # queue in reverse order
                    sweep_steps.insert(current_sweep_step, ["straight", False, [distance(self.xyz, HOMEPOINT), True]])
                    sweep_steps.insert(current_sweep_step, ["turn", False, [anglediff, True]])
                    #TODO: to go to homebase, measure angle and distance and then go straight instead of following localization, ignore all vslam info
                else:
                    print("VSLAM found after stepping back")
            except VSLAMFoundException as e:
                print("VSLAM found on way to base")
                self.returning_to_base = False
                current_sweep_step += 1
            except VSLAMEnabledException as e:
                print(self.sim_time, "VSLAM: mapping re-enabled")
                # rover often loses precision after pausing, go back to center to re-localize
                # sweep_steps.insert(current_sweep_step, ["goto", [HOMEPOINT, self.default_effort_level, False]]) # can't go every time
                wait_for_mapping = False
            except VSLAMDisabledException as e:
                print(self.sim_time, "VSLAM: mapping disabled, waiting")
                self.send_speed_cmd(0.0, 0.0)
                wait_for_mapping = True
            except LidarCollisionException as e: #TODO: long follow of obstacle causes loss, go along under steeper angle
                print(self.sim_time, "Lidar")
                if distance(self.xyz, pos) < ARRIVAL_TOLERANCE:
                    current_sweep_step += 1
                    continue
                sweep_steps.insert(current_sweep_step, ["lidar", True, []])
            except VirtualBumperException as e:
                print(self.sim_time, "Bumper")
                if distance(self.xyz, pos) < ARRIVAL_TOLERANCE:
                    current_sweep_step += 1
                    continue
                sweep_steps.insert(current_sweep_step, ["rock", True, []])
                sweep_steps.insert(current_sweep_step, ["straight", True, [-1, False]])

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
        wait_for_mapping = False
        while current_sweep_step < len(sweep_steps) and self.sim_time - start_time < timedelta(minutes=60):
            ex = None
            angle_diff = 0
            speed = 0
            pos = None
            try:
                while wait_for_mapping:
                    self.wait(timedelta(seconds=1))

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
            except VSLAMEnabledException as e:
                print(self.sim_time, "VSLAM: mapping re-enabled")
                wait_for_mapping = False
            except VSLAMDisabledException as e:
                print(self.sim_time, "VSLAM: mapping disabled, waiting")
                self.send_speed_cmd(0.0, 0.0)
                wait_for_mapping = True
            except VSLAMLostException as e:
                print(self.sim_time, "VSLAM lost")
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
                print(self.sim_time, "VSLAM found")
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

        self.set_cam_angle(0.1)
        self.set_light_intensity("0.4")
        self.set_brakes(False)

        def set_homebase_found(response):
            self.vslam_reset_at = self.sim_time

        try:
            self.wait_for_init()

            try:
                self.turn(math.radians(360), timeout=timedelta(seconds=20))
            except ChangeDriverException as e:
                pass
            except BusShutdownException:
                raise
            except:
                pass

            self.processing_plant_found = True
            self.send_speed_cmd(0.0, 0.0)
            self.set_cam_angle(-0.1)
            self.wait(timedelta(milliseconds=500))
            self.send_request('vslam_reset', set_homebase_found)

            while not self.true_pose:
                self.update()
            self.wait(timedelta(seconds=2)) # wait to receive first poses after true_pose before acting upon the current location

            # go to the nearest point on the circle?


            #self.stripe_to_obstacle_pattern()
            self.circular_pattern()

        except BusShutdownException:
            pass

        print(self.sim_time, "FINISHED")

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Space Robotics Challenge 2')
    args = parser.parse_args()

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
