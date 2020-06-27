"""
  Moon Rover Driver
"""

# source: (limited access)
#   https://gitlab.com/scheducation/srcp2-competitors/-/wikis/Documentation/API/Simulation_API

# Excavator Arm and Bucket
#  excavator_n/bucket_info
#  excavator_n/mount_joint_controller/command
#  excavator_n/basearm_joint_controller/command
#  excavator_n/distalarm_joint_controller/command
#  excavator_n/bucket_joint_controller/command

# Info
#  /excavator_n/bucket_info

# /name/joint_states  sensor_msgs/JointStates
# basearm_joint
# bucket_joint
# distalarm_joint
# mount_joint

from datetime import timedelta

import math
from osgar.lib.mathex import normalizeAnglePIPI

from osgar.node import Node
from moon.vehicles.rover import Rover

def angle_distance(alpha, beta):
    phi = abs(beta - alpha) % (2*math.pi)
    if phi > math.pi:
        phi = 360 - phi
    return phi

def rad_close(a,b):
    return [angle_distance(a[i],b[i]) < 0.1 for i in range(len(a))]

def rad_array_close(a, b):
    res = all(rad_close(a,b))
    return bool(res)

class Excavator(Rover):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cmd', 'bucket_cmd')
        # TODO: account for working on an incline

        self.target_arm_position = None
        self.current_arm_position = None

        self.bucket_status = None
        self.scoop_time = None
        self.execute_bucket_queue = []
        self.arm_joint_names = [b'mount_joint', b'basearm_joint', b'distalarm_joint', b'bucket_joint']
        self.bucket_scoop_sequence = (
            # [<seconds to execute>, [mount, base, distal, bucket]]
            [12, [-0.6, -0.8, 3.2], [-0.47, -0.26, 3.2]], # get above scooping position
            [4, [ 0.4, 1.0, 1.9], [0.55, 0.73, 1.85]], # lower to scooping position
            [2, [ 0.4, 1.0, 3.2], [0.6, 0.63, 3.04]], # scoop volatiles
            [8, [ -0.6, -0.8, 3.9], [-0.48, -0.35, 3.85]] # lift up bucket with volatiles
            )
        self.bucket_drop_sequence = (
            [12, [-0.6, -0.8, 3.9], [-0.48, -0.35, 3.85]], # turn towards dropping position
            [4, [-0.3, -0.8, 3.9], [-0.13, -0.38, 3.84]], # extend arm
            [4, [-0.3, -0.8, 0], [-0.12, -0.26, 0.02]], # drop
            [4, [-0.6, -0.8, 3.2], [-0.47, -0.26, 3.2]] # back to neutral/travel position
        )
        self.bucket_last_status_timestamp = None

    def send_bucket_position(self, bucket_params):
        mount, basearm, distalarm, bucket = bucket_params
        s = '%f %f %f %f\n' % (mount, basearm, distalarm, bucket)
        self.publish('bucket_cmd', bytes('bucket_position ' + s, encoding='ascii'))

    def on_bucket_info(self, data):
        self.bucket_status = data

    def on_bucket_dig(self, data):
        dig_angle, queue_action = data
        dig = [[duration, [dig_angle, *step], [dig_angle, *target]] for duration, step, target in self.bucket_scoop_sequence]
        if queue_action == 'reset':
            self.execute_bucket_queue = dig
            self.scoop_time = None
        elif queue_action == 'append':
            self.execute_bucket_queue += dig
        elif queue_action == 'prepend':
            self.execute_bucket_queue = dig + self.execute_bucket_queue
        else:
            assert False, "Dig command"

    def on_bucket_drop(self, data):
        drop_angle, queue_action = data
        drop = [[duration, [drop_angle, *step], [drop_angle, *target]] for duration, step, target in self.bucket_drop_sequence]
        if queue_action == 'reset':
            self.execute_bucket_queue = drop
            self.scoop_time = None
        elif queue_action == 'append':
            self.execute_bucket_queue += drop
        elif queue_action == 'prepend':
            self.execute_bucket_queue = drop + self.execute_bucket_queue
        else:
            assert False, "Drop command"

    def on_joint_position(self, data):
        super().on_joint_position(data)
        self.current_arm_position = [data[self.joint_name.index(n)] for n in self.arm_joint_names]

    def update(self):
        channel = super().update()

        # TODO: on a slope one should take into consideration current pitch and roll of the robot
        if self.time is not None:
            if (
                    len(self.execute_bucket_queue) > 0 and
                    (
                        self.scoop_time is None or
                        self.time > self.scoop_time or
                        self.target_arm_position is None or
                        rad_array_close(self.target_arm_position, self.current_arm_position)
                     )
            ):
                duration, bucket_params, bucket_targets = self.execute_bucket_queue.pop(0)
                self.target_arm_position = bucket_targets
#                print ("bucket_position %f %f %f " % (bucket_params[0], bucket_params[1],bucket_params[2]))
                self.send_bucket_position(bucket_params)
                self.scoop_time = self.time + timedelta(seconds=duration)

        # print status periodically - location and content of bucket if any
        if self.time is not None:
            if self.bucket_last_status_timestamp is None:
                self.bucket_last_status_timestamp = self.time
            elif self.time - self.bucket_last_status_timestamp > timedelta(seconds=8):
                self.bucket_last_status_timestamp = self.time
                if self.bucket_status is not None and self.bucket_status[1] != 100:
                    print ("Bucket content: Type: %s idx: %d mass: %f" % (self.bucket_status[0], self.bucket_status[1], self.bucket_status[2]))



        return channel


# vim: expandtab sw=4 ts=4
