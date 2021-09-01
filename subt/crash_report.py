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
        self.verbose = False

    def on_rgbd(self, data):
        self.buf.append(data)

    def on_acc(self, data):
        vec = [x/1000.0 for x in data]
        value = abs(distance3D(vec, [0, 0, 0]) - 9.81)
        if value >= self.threshold:
            if self.verbose:
                print(f'acc trigger: {value} (buf size = {len(self.buf)})')
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
    from osgar.logger import LogReader, LogWriter, lookup_stream_id, lookup_stream_names

    parser = argparse.ArgumentParser("Analyze crash_rgbd data")
    parser.add_argument('logfile', help='path to logfile')
    parser.add_argument('--out', help='custom output (default [logfile]-crash.log)')
    parser.add_argument('--verbose', '-v', help="verbose mode", action='store_true')
    args = parser.parse_args()

    outfile = args.out
    if outfile is None:
        assert args.logfile.endswith('.log')
        outfile = args.logfile[:-4] + '-crash.log'
    print('Crash outfile:', outfile)

    crash_stream_id = lookup_stream_id(args.logfile, 'black_box.crash_rgbd')
    pose3d_stream_id = lookup_stream_id(args.logfile, 'fromrospy.pose3d')

    def nearest_pose(poses, pose3d):
        xyz_arr = np.array([xyz for xyz, quat in poses])
        xyz_diff = xyz_arr - pose3d[0]
        dist = np.linalg.norm(xyz_diff, axis=1)
        index = np.argmin(dist)
        return index

    stream_names = lookup_stream_names(args.logfile)

    def get_camera_name_id(camera_pose):
        x, y = camera_pose[0][:2]
        if y == 0:
            camera_direction = 'front' if x > 0 else 'rear'
        elif y < 0:
            camera_direction = 'right'
        else:  # y > 0
            camera_direction = 'left'
        return stream_names.index(f'fromrospy.rgbd_{camera_direction}') + 1

    poses = []
    crash_time = None
    with LogReader(args.logfile,
               only_stream_id=[pose3d_stream_id, crash_stream_id]) as logreader, LogWriter(
            filename=outfile, start_time=logreader.start_time) as f:
        for name in stream_names:
            f.register(name)
        for time, stream, raw_bytes in logreader:
            data = deserialize(raw_bytes)
            if stream == pose3d_stream_id:
                poses.append(data)
                crash_time = None
            elif stream == crash_stream_id:
                robot_pose, camera_pose, rgb_compressed, depth_compressed = data
                if crash_time is None:
                    crash_time = time
                    if args.verbose:
                        print(f'------------- {crash_time} -------------')
                if args.verbose:
                    print(nearest_pose(poses, robot_pose), camera_pose[0])
                f.write(get_camera_name_id(camera_pose), raw_bytes, dt=time)

# vim: expandtab sw=4 ts=4
