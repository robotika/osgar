"""
    Quick check of robot logfiles.
"""
import os

import numpy as np
from osgar.logger import LogReader, lookup_stream_id


# Relevant streams, expected min frequency and expected max gap between msg.
g_relevant_streams = {
    "kloubak":{
        "kloubak.encoders": [19, 0.5],
        "kloubak.can": [79, 0.5],
        "kloubak.joint_angle": [19, 0.5],
        "lidar.scan": [4, 2],
        "lidar_back.scan": [4, 2],
        "camera.raw": [4, 2],
        "camera_back.raw": [4, 2],
        "imu.orientation": [14, 0.5],
        "imu.rotation": [14, 0.5],
        "wifi.wifiscan": [0.7, 2],
        "gas_detector.co2": [1, 3],
        "rosmsg.image": [1, 3],
        "rosmsg.pose2d": [10, 1],
        "rosmsg.depth": [1, 3],
        "rosmsg_rear.image": [1, 3],
        "rosmsg_rear.pose2d": [10, 1],
        "rosmsg_rear.depth": [1, 3]
    },
    "skiddy":{}  # TODO
}


def main(logfile, streams):
    warning_event = False
    base_logname = os.path.basename(logfile)
    print("\n" + base_logname)
    print("-"*60)
    relevant_streams_id = [lookup_stream_id(logfile, name) for name in streams.keys()]
    stat_list = [np.array([])] * len(relevant_streams_id)
    with LogReader(logfile, only_stream_id=relevant_streams_id) as log:
        for timestamp, stream_id, data in log:
            time_in_sec = timestamp.total_seconds()
            item_id = relevant_streams_id.index(stream_id)
            stat_list[item_id] = np.append(stat_list[item_id], time_in_sec)

    for arr, name in zip(stat_list, streams):
        gaps_event = False
        if len(arr) <= 1:
            print(f'\n{name:>{20}}\t{"No data or only one msg received!!!"}')
            warning_event = True
        else:
            # average frequencies
            average_freq = len(arr) / (arr[-1] - arr[0])  # average number of msg per second
            expected_freq = streams[name][0]
            acceptable_gap = streams[name][1]
            num_gaps = len(np.where(np.diff(arr)>acceptable_gap)[0])
            if num_gaps > 0:
                max_gap = max(np.diff(arr))
                print(f'\n{name:>{20}}\t{"number of gaps %d" %num_gaps}\t{"max gap %.2f s" %max_gap}\t{"acceptable gap %.2f s" %acceptable_gap}')
                gaps_event = True
                warning_event = True

            if average_freq < expected_freq:
                if not gaps_event:
                    print("")
                print(f'{name:>{20}}\t{"received %.1f Hz" % average_freq}\t{"expected %.1f Hz" % expected_freq}')
                warning_event = True

    if not warning_event:
        print("log OK")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='filename of stored file or directory')
    parser.add_argument('--robot', help='robot name', default="kloubak")
    parser.add_argument('--log-prefix', help='prefix of a logname, e.g. kloubak2-subt-estop-lora-jetson')
    args = parser.parse_args()

    relevant_streams = g_relevant_streams[args.robot]
    logfile = args.logfile
    if os.path.isdir(logfile):
        if args.log_prefix:
            prefix = args.log_prefix
        else:
            print("WARNING: logname prefix is not defined. Used robot name.")
            prefix = args.robot

        logname_list = os.listdir(logfile)
        for logname in logname_list:
            if prefix not in logname:
                continue
            logname_path = os.path.join(logfile, logname)
            main(logname_path, relevant_streams)

    else:
        main(logfile, relevant_streams)
