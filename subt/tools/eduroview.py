import sys
from osgar.tools import lidarview

if __name__ == "__main__":
    args = [
        "--pose2d", "app.pose2d",
        "--lidar", "lidar.scan",
        "--camera", "camera.raw",
        "--keyframes", "detector.artf"
    ] + sys.argv[1:]
    lidarview.main(args)
