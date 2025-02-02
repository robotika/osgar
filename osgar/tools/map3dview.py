"""
  Visualize 3D local map(s) from OAK-D camera [later also with odometry and map creation]
"""
from datetime import timedelta

import open3d as o3d
import numpy as np

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def get_points(depth):
    # Create point cloud
    h, w = depth.shape
    fx = fy = 600  # Focal length in pixels (adjust as necessary)
    cx, cy = w // 2, h // 2  # Optical center

    points = []

    for v in range(h):
        for u in range(w):
            z = depth[v, u] / 1000.0  # Convert to meters
            if z > 0:  # Only consider valid points
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])
    return points


def visualize3d(logfile, depth_name, camera_name):
    depth_stream = lookup_stream_id(logfile, depth_name)
    camera_stream = lookup_stream_id(logfile, camera_name)
    only_stream = [depth_stream, camera_stream]
    buf = None
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)
    # https://www.open3d.org/docs/latest/python_api/open3d.visualization.ViewControl.html
    ctr = vis.get_view_control()
    ctr.set_front([ 0.038523686988841843, -0.11245546606954208, -0.99290971074507473 ])
    ctr.set_lookat([ -0.21639217215263715, 1.0339043338883831, 14.773016440432482 ])
    ctr.set_up([ -0.024914348639606877, -0.99344650502891818, 0.11154961621490113 ])
    ctr.set_zoom(0.3)

    with LogReader(logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            if timestamp < timedelta(seconds=4.5):
                continue
            buf = deserialize(data)
            if stream_id == depth_stream:
                print(timestamp)
                # Create Open3D point cloud object
                pcd.points = o3d.utility.Vector3dVector(get_points(buf))
                vis.update_geometry(pcd)
                ctr.set_front([0.038523686988841843, -0.11245546606954208, -0.99290971074507473])
                ctr.set_lookat([-0.21639217215263715, 1.0339043338883831, 14.773016440432482])
                ctr.set_up([-0.024914348639606877, -0.99344650502891818, 0.11154961621490113])
                ctr.set_zoom(0.3)
                if not vis.poll_events():
                    break
                vis.update_renderer()
    vis.destroy_window()

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
