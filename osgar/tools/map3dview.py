"""
  Visualize 3D local map(s) from OAK-D camera [later also with odometry and map creation]
"""
from datetime import timedelta

import open3d as o3d
import numpy as np

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def visualize3d(logfile, depth_name, camera_name):
    depth_stream = lookup_stream_id(logfile, depth_name)
    camera_stream = lookup_stream_id(logfile, camera_name)
    only_stream = [depth_stream, camera_stream]
    buf = None
    with LogReader(logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            if timestamp < timedelta(seconds=4.5):
                continue
            buf = deserialize(data)
            if stream_id == depth_stream:
                break

    if buf is not None:
        depth = buf
        # Create point cloud
        h, w = depth.shape
        fx = fy = 600  # Focal length in pixels (adjust as necessary)
        cx, cy = w // 2, h // 2  # Optical center

        points = []
        colors = []

        for v in range(h):
            for u in range(w):
                z = depth[v, u] / 1000.0  # Convert to meters
                if z > 0:  # Only consider valid points
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])
#                    colors.append(color[v, u] / 255.0)  # Normalize color values

        # Create Open3D point cloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
#        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Visualize the point cloud
        o3d.visualization.draw_geometries([pcd])


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--depth', help='stream ID or name', default='oak.depth')
    parser.add_argument('--camera', help='optional color camera stream ID or name', default='oak.color')
    args = parser.parse_args()

    visualize3d(args.logfile, args.depth, args.camera)


if __name__ == "__main__":
    main()
