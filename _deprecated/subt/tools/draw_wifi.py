import sys
import os
import numpy as np
from matplotlib import pyplot as plt

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def draw_wifi(logs_dir):
    path_logs = []
    if os.path.isdir(logs_dir):
        sub_dirs = os.listdir(logs_dir)
        for robot_dir in sub_dirs:
            robot_dir_path = os.path.join(logs_dir, robot_dir)
            if os.path.isdir(robot_dir_path):
                logs = os.listdir(robot_dir_path)
                for log_name in logs:
                    path_logs.append(os.path.join(robot_dir_path, log_name))
            else:
                path_logs.append(robot_dir_path)  # logs_dir is for one specific robot
    else:
        path_logs.append(logs_dir)

    wifi_pose = []
    co2_pose = []
    for path_log in path_logs:
        print(path_log)
        try:
            only_stream_wifi = lookup_stream_id(path_log, "wifi.wifiscan")
            only_stream_co2 = lookup_stream_id(path_log, "gas_detector.co2")
            only_stream_pose = lookup_stream_id(path_log, "app.pose2d")
            with LogReader(path_log, only_stream_id=[only_stream_wifi, only_stream_co2, only_stream_pose]) as log:
                x = y = 0
                for timestamp, stream_id, data_raw in log:
                    if stream_id == only_stream_pose:
                        x, y, __  = deserialize(data_raw)  # last pose2d
                    elif stream_id == only_stream_wifi:
                        wifi = deserialize(data_raw)
                        for item in wifi:
                            if "PhoneArtifac" in item[0]:
                                wifi_pose.append([x, y, item[1]])
                    elif stream_id == only_stream_co2:
                        co2_pose.append([x, y, deserialize(data_raw)])
        except:
            pass
            #print("End..")  # exception during log reading

    fig = plt.figure(figsize=(12, 16))
    fig.subplots_adjust(left=0.1, bottom=0.1, right=0.9, top=0.9 )
    if len(wifi_pose) != 0:
        ax = fig.add_subplot(211)
        wifi_ar = np.array(wifi_pose)
        im = ax.scatter(wifi_ar[:,0]/1000, wifi_ar[:,1]/1000, c=wifi_ar[:,2], cmap=plt.cm.jet)
        ax.set_title("WIFI")
        ax.axis('equal')
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        cb = fig.colorbar(im, ax=ax, orientation="vertical", pad=.15, aspect=60)
        cb.ax.tick_params(labelsize=10)
        cb.set_label("Wifi", rotation=0)
    else:
        print("No wifi")

    if len(co2_pose) != 0:
        co2_ar = np.array(co2_pose)
        ax2 = fig.add_subplot(212)
        im_co2 = ax2.scatter(co2_ar[:, 0] / 1000, co2_ar[:, 1] / 1000, c=co2_ar[:, 2], cmap=plt.cm.jet)
        ax2.set_title("CO2")
        ax2.axis('equal')
        ax2.set_xlabel("x (m)")
        ax2.set_ylabel("y (m)")
        cb2 = fig.colorbar(im_co2, ax=ax2, orientation="vertical", pad=.15, aspect=60)
        cb2.ax.tick_params(labelsize=10)
        cb2.set_label("co2", rotation=0)
    else:
        print("No co2")

    plt.show()

if __name__ == "__main__":
    if len(sys.argv) == 2:
        draw_wifi(sys.argv[1])
    else:
        draw_wifi(".")
