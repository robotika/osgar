import argparse
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize
from osgar.lib.quaternion import euler_zyx

def get_optical_flow_angle(img, prev_gray):
    curr_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    da = 0.0
    
    if prev_gray is not None:
        prev_pts = cv2.goodFeaturesToTrack(prev_gray,
                                           maxCorners=200,
                                           qualityLevel=0.01,
                                           minDistance=30,
                                           blockSize=3)
                                           
        if prev_pts is not None and len(prev_pts) > 0:
            curr_pts, status, err = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray, prev_pts, None)
            idx = np.where(status == 1)[0]
            if len(idx) > 0:
                prev_pts = prev_pts[idx]
                curr_pts = curr_pts[idx]
                
                m, inliers = cv2.estimateAffinePartial2D(prev_pts, curr_pts)
                if m is not None:
                    # m[1, 0] is sin(da), m[0, 0] is cos(da)
                    da = np.arctan2(m[1, 0], m[0, 0])
    
    return da, curr_gray

def main():
    log_file = "test-image-rotator-260518_190752.log"
    color_id = lookup_stream_id(log_file, "oak.color")
    imu_id = lookup_stream_id(log_file, "oak.orientation_list")

    imu_angles = []
    of_angles = []

    last_orientation = None
    prev_gray = None
    accumulated_of_angle = 0.0

    with LogReader(log_file, only_stream_id=[color_id, imu_id]) as log:
        for dt, channel, data in log:
            data = deserialize(data)
            if channel == imu_id:
                if len(data) > 0:
                    last_orientation = data[-1][2:6]
            elif channel == color_id:
                img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_ANYCOLOR)
                if img is None:
                    continue
                
                # IMU Angle
                imu_angle = 0.0
                if last_orientation is not None:
                    yaw, pitch, roll = euler_zyx(last_orientation)
                    imu_angle = roll # Without multiplier, just raw roll
                
                imu_angles.append(math.degrees(imu_angle))

                # Optical Flow Angle
                da, prev_gray = get_optical_flow_angle(img, prev_gray)
                accumulated_of_angle += da
                of_angles.append(math.degrees(accumulated_of_angle))

    plt.figure(figsize=(10, 6))
    plt.plot(imu_angles, label='IMU Roll (raw)')
    plt.plot(of_angles, label='Optical Flow Accumulated Angle')
    
    # Let's also plot the optical flow angle with negative sign if it makes it match better
    # and maybe try to align their starting points
    if len(imu_angles) > 0 and len(of_angles) > 0:
        offset = imu_angles[0] - of_angles[0]
        plt.plot([a + offset for a in of_angles], label='Optical Flow (offset to match IMU start)', linestyle='--')
        
        offset_neg = imu_angles[0] - (-of_angles[0])
        plt.plot([-a + offset_neg for a in of_angles], label='-Optical Flow (offset to match IMU start)', linestyle=':')

    plt.xlabel('Frame')
    plt.ylabel('Angle (degrees)')
    plt.legend()
    plt.title('Comparison of Image Rotation Methods')
    plt.grid(True)
    plt.savefig('rotation_comparison.png')
    
    print(f"Processed {len(imu_angles)} frames.")
    print("Saved comparison plot to rotation_comparison.png")

if __name__ == "__main__":
    main()
