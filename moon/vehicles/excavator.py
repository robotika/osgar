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

def rad_close(a,b):
    return [abs(normalizeAnglePIPI(x-y)) < 0.1 for x,y in zip(a,b)]

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

        self.scoop_time = None
        self.execute_bucket_queue = []
        self.arm_joint_names = [b'mount_joint', b'basearm_joint', b'distalarm_joint', b'bucket_joint']
        self.bucket_scoop_sequence = (
            # [<seconds to execute>, [mount, base, distal, bucket]]
            # note, even though target angles are in range, the movement may be obstructed by another part of the robot (e.g, camera)
            [20, [-0.6,  -0.8, 3.2]], # get above scooping position #TODO: no need to move this high, however, need to swing through front of the robot not to hit hauler
            [20, [ 0.66, -0.6, 3.2]], # lower to scooping position
            [20, [ 0.66,   0.8, 2.5]], # scoop volatiles
            [20, [-0.6,  -0.2, 3.92]] # lift up bucket with volatiles
            )
        self.bucket_drop_sequence = (
            [20, [-0.6, -0.2, 3.92]], # turn towards dropping position
            [10, [0, -0.8, 3.92]], # extend arm
            [10, [-0.3, -0.8, 3]], # drop
            [20, [-0.6, -0.8, 3.2]] # back to neutral/travel position
        )
        self.bucket_last_status_timestamp = None

    def send_bucket_position(self, bucket_params):
        mount, basearm, distalarm, bucket = bucket_params
        if 0.4 <= abs(mount) <= 1.25 or 1.85 <= abs(mount) <= 2.75:
            distalarm = min(0.15, distalarm)
        self.target_arm_position = [mount, basearm, distalarm, bucket]
        s = '%f %f %f %f\n' % (mount, basearm, distalarm, bucket)
        self.publish('bucket_cmd', bytes('bucket_position ' + s, encoding='ascii'))

    def on_bucket_dig(self, data):
        dig_angle, queue_action = data
        dig = [[duration, [dig_angle, *step]] for duration, step in self.bucket_scoop_sequence]
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
        drop = [[duration, [drop_angle, *step]] for duration, step in self.bucket_drop_sequence]
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
        if self.sim_time is not None:
            if (
                    len(self.execute_bucket_queue) > 0 and
                    (
                        self.scoop_time is None or
                        self.sim_time > self.scoop_time or
                        self.target_arm_position is None or
                        rad_array_close(self.target_arm_position, self.current_arm_position)
                     )
            ):
                duration, bucket_params = self.execute_bucket_queue.pop(0)
#                print ("bucket_position %f %f %f " % (bucket_params[0], bucket_params[1],bucket_params[2]))
                self.send_bucket_position(bucket_params)
                self.scoop_time = self.sim_time + timedelta(seconds=duration)

        return channel


# vim: expandtab sw=4 ts=4
