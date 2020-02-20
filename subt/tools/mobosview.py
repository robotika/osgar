import sys
from osgar.tools import lidarview

if __name__ == "__main__":
    args = [
        "--pose2d", "app.pose2d",
        "--lidar", "rosmsg.scan",
        "--camera", "rosmsg.image",
        "--camera2", "rosmsg.depth",
    ] + sys.argv[1:]
    lidarview.main(args)

