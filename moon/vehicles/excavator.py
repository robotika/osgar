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

from datetime import timedelta

import math
from osgar.lib.mathex import normalizeAnglePIPI

from osgar.node import Node
from moon.rover import Rover

class Excavator(Rover):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cmd', 'bucket_cmd')
        # TODO: account for working on an incline

        self.bucket_status = None
        self.scoop_time = None
        self.scoop_index = 0
        self.bucket_scoop_part = True # True for scoop part, False for drop part
        self.bucket_scoop_sequence = (
            # [<seconds to execute>, [mount, base, distal, bucket]] 
            [12, [-0.6, -0.8, 3.2]], # get above scooping position
            [4, [ 0.4, 1.0, 1.9]], # lower to scooping position
            [2, [ 0.4, 1.0, 3.2]], # scoop volatiles
            [8, [ -0.6, -0.8, 3.9]] # lift up bucket with volatiles
            )
        self.bucket_drop_sequence = (
            [12, [-0.6, -0.8, 3.9]], # turn towards dropping position
            [4, [-0.3, -0.8, 3.9]], # extend arm
            [4, [-0.3, -0.8, 0]], # drop
            [4, [-0.6, -0.8, 3.2]] # back to neutral/travel position
        )
        self.bucket_last_status_timestamp = None
        
    def send_bucket_position(self, bucket_params):
        mount, basearm, distalarm, bucket = bucket_params
        s = '%f %f %f %f\n' % (mount, basearm, distalarm, bucket)
        self.publish('bucket_cmd', bytes('bucket_position ' + s, encoding='ascii'))
        
    def on_bucket_info(self, data):
        self.bucket_status = data
        

    def update(self):

        # this is just a demo of scooping periodically;
        # note that the angles may be off and should take into consideration current pitch and roll of the robot
        if self.time is not None:
            if self.scoop_time is None or self.time > self.scoop_time:
                if self.bucket_scoop_part:
                    duration, bucket_params = self.bucket_scoop_sequence[self.scoop_index]
                    target_angle = 3.4 # demo scooping towards the back of the vehicle
                else:
                    duration, bucket_params = self.bucket_drop_sequence[self.scoop_index]
                    target_angle = 0.2 # demo dropping towards the rear of the vehicle
#                print ("bucket_position %f %f %f " % (bucket_params[0], bucket_params[1],bucket_params[2]))
                self.send_bucket_position([target_angle, *bucket_params])
                self.scoop_time = self.time + timedelta(seconds=duration)
                if self.scoop_index + 1 == len(self.bucket_scoop_sequence if self.bucket_scoop_part else self.bucket_drop_sequence):
                    self.scoop_index = 0
                    self.bucket_scoop_part = not self.bucket_scoop_part
                else:
                    self.scoop_index += 1
                    

        # print status periodically - location and content of bucket if any
        if self.time is not None:
            if self.bucket_last_status_timestamp is None:
                self.bucket_last_status_timestamp = self.time
            elif self.time - self.bucket_last_status_timestamp > timedelta(seconds=8):
                self.bucket_last_status_timestamp = self.time
                if self.bucket_status is not None and self.bucket_status[1] > 0:
                    print ("Bucket content: Type: %s idx: %d mass: %f" % (self.bucket_status[0], self.bucket_status[1], self.bucket_status[2]))
 

        
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

        return channel


# vim: expandtab sw=4 ts=4
