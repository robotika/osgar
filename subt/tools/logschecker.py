"""
    Quick check of kloubak logfiles.
"""
import os

import numpy as np
from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


# Relevant streams, expected min frequency and expected max gap between msg.
g_relevant_streams = {  # used just for kloubak robot
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
        "gas_detector.co2": [1, 3]
}


def main(logfile, streams):
    warning_event = False
    base_logname = os.path.basename(logfile)
    print("\n" + base_logname)
    print("-"*60)
    relevant_streams_id = [lookup_stream_id(logfile, name) for name in streams.keys()]
    stat_list = [np.array([])] * len(relevant_streams_id)
    encoder_stream_id = lookup_stream_id(logfile, "kloubak.encoders")
    prev_time_in_sec = None
    time_diff = None
    with LogReader(logfile, only_stream_id=relevant_streams_id) as log:
        for timestamp, stream_id, data in log:
            time_in_sec = timestamp.total_seconds()
            item_id = relevant_streams_id.index(stream_id)
            stat_list[item_id] = np.append(stat_list[item_id], time_in_sec)

            if stream_id == encoder_stream_id:
                encoders = deserialize(data)
                for enc in encoders:
                    if enc > abs(100):
                        if prev_time_in_sec is not None:
                            time_diff = time_in_sec - prev_time_in_sec
                        print(f"\nEncoders: {encoders}, time diff: {time_diff}, at {timestamp}")
                prev_time_in_sec = time_in_sec

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
    parser.add_argument('--keyword', help='keyword in a logname, e.g. kloubak2-subt-estop-lora-jetson')
    args = parser.parse_args()

    relevant_streams = g_relevant_streams
    logfile = args.logfile
    if os.path.isdir(logfile):
        if args.keyword:
            keyword = args.keyword
        else:
            print("WARNING: logname keyword is not defined. Used robot name: kloubak.")
            keyword = "kloubak"

        logname_list = os.listdir(logfile)
        for logname in logname_list:
            if keyword not in logname:
                continue
            logname_path = os.path.join(logfile, logname)
            main(logname_path, relevant_streams)

    else:
        main(logfile, relevant_streams)
