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
ORIGIN_STREAM = 'rosmsg.origin'


def evaluate_poses(poses, gt_poses, time_step_sec=1):
    if len(poses) == 0 or len(gt_poses) == 0:
        return []
    time_limit = max(poses[0][0], gt_poses[0][0])
    end_time = min(poses[-1][0], gt_poses[-1][0])

    i, j = 0, 0
    arr = []
    while time_limit <= end_time:
        while poses[i][0] < time_limit:
            i += 1
        while gt_poses[j][0] < time_limit:
            j += 1
        dist = distance3D(poses[i][1:], gt_poses[j][1:])
        diff = [a - b for a, b in zip(poses[i][1:], gt_poses[j][1:])]
        arr.append((time_limit, dist, *diff))
        time_limit += time_step_sec
    return arr


def ign2arr(ign_poses, robot_name):
    """Convert Ignition state poses into array"""
    arr = []
    prev_seconds = None
    for timestamp, data in ign_poses:
        if robot_name in data:
            pos = data[robot_name]
            if prev_seconds != timestamp.seconds:
                # store only position every simulation second
                if prev_seconds is not None:
                    # ignore the first sample which is probably not on time boundary (we cannot prove it)
                    arr.append((int(round(timestamp.total_seconds())), pos.x, pos.y, pos.z))
                prev_seconds = timestamp.seconds
    return arr


def osgar2arr(poses):
    """drop orientation information from poses"""
    arr = []
    for time_sec, data in poses:
        arr.append((time_sec, *data[0]))
    return arr


def read_pose3d(filename, pose3d_name, seconds=MAX_SIMULATION_DURATION):
    stream_id = lookup_stream_id(filename, pose3d_name)
    sim_time_id = lookup_stream_id(filename, SIM_TIME_STREAM)
    poses = []
    sim_time = None
    pose_sim_time = None  # sim_time, but valid only for the first pose after time change
    for dt, channel, data in LogReader(filename, only_stream_id=[stream_id, sim_time_id]):
        value = deserialize(data)
        if len(poses) > seconds:
            break
        if channel == sim_time_id:
            if sim_time != value:
                sim_time = value
                pose_sim_time = sim_time
        else:  # pose3d
            if pose_sim_time is not None:
                poses.append((pose_sim_time, value))
                pose_sim_time = None
    return poses


def autodetect_name(logfile):
    stream_id = lookup_stream_id(logfile, ORIGIN_STREAM)
    for dt, channel, data in LogReader(logfile, only_stream_id=stream_id):
        arr = deserialize(data)
        robot_name = arr[0].decode('ascii')
        return robot_name
    assert False, "Robot name autodetection failed!"


def main():
    import argparse
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('logfile', help='OSGAR logfile')
    parser.add_argument('--ign', help='Ignition "state.tlog"', required=True)
    parser.add_argument('--pose3d', help='Stream with pose3d data', default='app.pose3d')
    parser.add_argument('--name', help='Robot name, default is autodetect')
    parser.add_argument('--sec', help='duration of the analyzed part (seconds)',
                        type=float, default=MAX_SIMULATION_DURATION)
    parser.add_argument('--draw', help="draw debug results", action='store_true')
    args = parser.parse_args()

    robot_name = args.name
    if robot_name is None:
        robot_name = autodetect_name(args.logfile)
        print('Autodetected name:', robot_name)

    ground_truth = ign.read_poses(args.ign, seconds=args.sec)
    print('GT count:', len(ground_truth))

    pose3d = read_pose3d(args.logfile, args.pose3d, seconds=args.sec)
    print('Trace reduced count:', len(pose3d))

    tmp_poses = osgar2arr(pose3d)
    tmp_gt = ign2arr(ground_truth, robot_name=robot_name)
    print('GT seconds:', len(tmp_gt))
    arr = evaluate_poses(tmp_poses, tmp_gt)
    if len(arr) == 0:
        print('EMPTY OVERLAP!')
        if len(tmp_poses) > 0:
            print('poses:', tmp_poses[0][0], tmp_poses[-1][0])
        if len(tmp_gt) > 0:
            print('gt   :', tmp_gt[0][0], tmp_gt[-1][0])
    else:
        dist3d = [a[1] for a in arr]
        print(max(dist3d))
        assert min(dist3d) < 0.1, min(dist3d)  # the minimum should be almost zero for correct evaluation

    if args.draw:
        import matplotlib.pyplot as plt

        fig, ax1 = plt.subplots()
        ax1.set_ylabel('distance (m)')
        ax1.set_title(robot_name)

        x = [a[0] for a in arr]
        for index, label in enumerate(['dist3d', 'x', 'y', 'z']):
            y = [a[index + 1] for a in arr]
            ax1.plot(x, y, '-', linewidth=2, label=label)
        ax1.set_xlabel('sim time (s)')
        plt.legend()
        plt.show()

    return arr


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
