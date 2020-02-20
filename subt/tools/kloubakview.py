import sys
from osgar.tools import lidarview

if __name__ == "__main__":
    args = [
        "--pose2d", "app.pose2d",
        "--lidar", "lidar.scan",
        "--lidar2", "lidar_back.scan",
        "--camera", "camera.raw",
        "--camera2", "camera_back.raw",
        "--keyframes", "detector.artf",
        "--joint", "kloubak.joint_angle",
    ] + sys.argv[1:]
    lidarview.main(args)
