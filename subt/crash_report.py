"""
  Keep given buffer of RGBD images until crash happens (detected in accelerometers)
"""
import collections

from osgar.node import Node
from subt.trace import distance3D


class CrashReport(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("crash_rgbd")
        size = config.get('size', 100)  # keep N last records
        self.threshold = config.get('threshold', 100.0)  # i.e. 10g
        self.buf = collections.deque(maxlen=size)

    def on_rgbd(self, data):
        self.buf.append(data)

    def on_acc(self, data):
        vec = [x/1000.0 for x in data]
        value = abs(distance3D(vec, [0, 0, 0]) - 9.81)
        if value >= self.threshold:
            for rec in self.buf:
                self.publish('crash_rgbd', rec)
            self.buf.clear()

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unsupported channel
        return channel


if __name__ == "__main__":
    import argparse
    import numpy as np
    from osgar.lib.serialize import deserialize
    from osgar.logger import LogReader, lookup_stream_id

    parser = argparse.ArgumentParser("Analyze crash_rgbd data")
    parser.add_argument('logfile', help='path to logfile')
    args = parser.parse_args()

    crash_stream_id = lookup_stream_id(args.logfile, 'black_box.crash_rgbd')
    pose3d_stream_id = lookup_stream_id(args.logfile, 'fromrospy.pose3d')

    def nearest_pose(poses, pose3d):
        xyz_arr = np.array([xyz for xyz, quat in poses])
        xyz_diff = xyz_arr - pose3d[0]
        dist = np.linalg.norm(xyz_diff, axis=1)
        index = np.argmin(dist)
        return index

    poses = []
    crash_time = None
    with LogReader(args.logfile,
               only_stream_id=[pose3d_stream_id, crash_stream_id]) as logreader:
        for time, stream, data in logreader:
            data = deserialize(data)
            if stream == pose3d_stream_id:
                poses.append(data)
                crash_time = None
            elif stream == crash_stream_id:
                robot_pose, camera_pose, rgb_compressed, depth_compressed = data
                if crash_time is None:
                    crash_time = time
                    print(f'------------- {crash_time} -------------')
                print(nearest_pose(poses, robot_pose), camera_pose[0])

# vim: expandtab sw=4 ts=4
