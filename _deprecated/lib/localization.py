#!/usr/bin/env python

import math


# pass as parameters? configuration file??
FRONT_REAR_DIST = 1.3
LEFT_WHEEL_DIST_OFFSET = 0.4  # from central axis


def normalizeAnglePIPI( angle ):
  "normalize only within 2PI error"
  if angle < -math.pi:
    angle += 2*math.pi
  if angle > math.pi:
    angle -= 2*math.pi
  return angle


def combine_poses( robotPose, sensorPose ):
    # used to be combinedPose
  return (
    robotPose[0] + sensorPose[0] * math.cos( robotPose[2] ) - sensorPose[1] * math.sin( robotPose[2] ),
    robotPose[1] + sensorPose[0] * math.sin( robotPose[2] ) + sensorPose[1] * math.cos( robotPose[2] ),
    normalizeAnglePIPI( robotPose[2] + sensorPose[2] ) )


class SimpleOdometry():
    def __init__( self, pose = (0,0,0) ):
        self.x = pose[0]
        self.y = pose[1]
        self.heading = pose[2]
        self.config = {
                'laser': (1.78, 0.0, 0.0),  # height 0.39
            }  # TODO external configuration
        self.global_map = [] 

    @classmethod
    def from_dict(cls, config_dict):
        ret = cls(pose=config_dict.get('pose', (0, 0, 0)))
        if 'cones' in config_dict:
            ret.global_map = config_dict['cones']
        return ret

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

    def eval_map_pose(self, ref_pose, source_id, data, verbose=False):
        assert source_id in self.config, source_id
        assert source_id == 'laser', source_id
        laser_pose = combine_poses(ref_pose, self.config[source_id])
        ret = 0.0
        for tick_angle, tick_dist, tick_width in data:
            angle = math.radians((tick_angle - 270) * 0.5)
            dist = tick_dist/1000.0
            x = laser_pose[0] + dist * math.cos(laser_pose[2] + angle)
            y = laser_pose[1] + dist * math.sin(laser_pose[2] + angle)
            for cx, cy in self.global_map:
                err = math.hypot(x - cx, y - cy)
                if err < 2.0:
                    if verbose:
                        print("eval_map_pose", err)
                    ret += err*err
        return ret

    def update_landmarks(self, source_id, data):
        dx, dy, da = 0.1, 0.1, math.radians(1.0)
        print(data)
        best_err = self.eval_map_pose(self.pose(), source_id, data, verbose=True)
        x, y, a = self.pose()
        poses = [(x-dx, y, a), (x+dx, y, a),
                 (x, y-dy, a), (x, y+dy, a),
                 (x, y, a-da), (x, y, a+da)]
        for i, ref in enumerate(poses):
            err = self.eval_map_pose(ref, source_id, data)
            if err < best_err:
                print(i, err, best_err)
                best_err = err
                self.x, self.y, self.heading = ref

# vim: expandtab sw=4 ts=4 

