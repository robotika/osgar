import sys
from osgar.tools import lidarview

if __name__ == "__main__":
    args = [
        "--pose3d", "from_jetson_front.pose3d",
        "--lidar", "fromros.scan360",
        "--deg","360",
        "--camera", "camera.raw",
        "--camera2", "camera_back.raw",
        "--keyframes", "from_jetson_rear.localized_artf",
        "--joint", "kloubak.joint_angle",
    ] + sys.argv[1:]
    lidarview.main(args, "kloubak")
