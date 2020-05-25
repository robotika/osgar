"""
  Validate pose3d against ground truth from ignition simulator
"""
import datetime

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize
import subt.tools.ignstate as ign
from subt.trace import distance3D


MAX_SIMULATION_DURATION = 3700  # 1hour with a small buffer

SIM_TIME_STREAM = 'rosmsg.sim_time_sec'


def evaluate_poses(poses, gt_poses, time_step_sec=1.0):
    time_limit = max(poses[0][0], gt_poses[0][0])
    end_time = min(poses[-1][0], gt_poses[-1][0])

    i, j = 0, 0
    arr = []
    while time_limit < end_time:
        while poses[i][0] < time_limit:
            i += 1
        while gt_poses[j][0] < time_limit:
            j += 1
        dist = distance3D(poses[i][1:], gt_poses[j][1:])
        arr.append(dist)
        time_limit += time_step_sec
    return arr


def ign2arr(ign_poses, robot_name):
    """Convert Ignition state poses into array"""
    arr = []
    for timestamp, data in ign_poses:
        if robot_name in data:
            pos = data[robot_name]
            arr.append((timestamp.total_seconds(), pos.x, pos.y, pos.z))
    return arr


def osgar2arr(poses):
    arr = []
    for timestamp, data in poses:
        arr.append((timestamp.total_seconds(), *data[0]))
    return arr


def read_pose3d(filename, pose3d_name, seconds=MAX_SIMULATION_DURATION):
    stream_id = lookup_stream_id(filename, pose3d_name)
    sim_time_id = lookup_stream_id(filename, SIM_TIME_STREAM)
    poses = []
    sim_time = 0
    for dt, channel, data in LogReader(filename, only_stream_id=[stream_id, sim_time_id]):
        value = deserialize(data)
        if sim_time > seconds:
            break
        if channel == sim_time_id:
            sim_time = value
        else:  # pose3d
            poses.append((datetime.timedelta(seconds=sim_time), value))
    return poses


def main():
    import argparse
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('logfile', help='OSGAR logfile')
    parser.add_argument('--ign', help='Ignition "state.tlog"', required=True)
    parser.add_argument('--pose3d', help='Stream with pose3d data', default='app.pose3d')
    parser.add_argument('--name', help='Robot name, default is autodetect')
    parser.add_argument('--sec', help='duration of the analyzed part (seconds)',
                        type=float, default=MAX_SIMULATION_DURATION)
    args = parser.parse_args()

    ground_truth = ign.read_poses(args.ign, seconds=args.sec)
    print('GT count:', len(ground_truth))

    pose3d = read_pose3d(args.logfile, args.pose3d, seconds=args.sec)
    print('Trace count:', len(pose3d))

    arr = evaluate_poses(osgar2arr(pose3d), ign2arr(ground_truth, robot_name=args.name))
    print(min(arr), max(arr))
    return arr


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
