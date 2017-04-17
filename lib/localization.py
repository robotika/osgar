#!/usr/bin/env python

import math


# pass as parameters? configuration file??
FRONT_REAR_DIST = 1.3
LEFT_WHEEL_DIST_OFFSET = 0.4  # from central axis


class SimpleOdometry():
    def __init__( self, pose = (0,0,0) ):
        self.x = pose[0]
        self.y = pose[1]
        self.heading = pose[2]

    def update_odometry(self, angle_left, dist_left, dist_right):
        dh = math.sin(angle_left) * dist_left / FRONT_REAR_DIST
        dist = math.cos(angle_left) * dist_left + LEFT_WHEEL_DIST_OFFSET * dh 
                    
        self.x += math.cos(self.heading) * dist
        self.y += math.sin(self.heading) * dist
        self.heading += dh

    def pose( self ):
        return (self.x, self.y, self.heading)

    def set_pose( self, pose ):
        (self.x, self.y, self.heading) = pose

    def updateGPS( self, gpsData ):
        pass 

# vim: expandtab sw=4 ts=4 

