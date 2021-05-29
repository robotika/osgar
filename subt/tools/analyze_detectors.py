"""evaluate detectors"""
import datetime
import os
import csv
import numpy as np
import cv2
from matplotlib import pyplot as plt

try:
    import torch
    import subt.artf_model
    from subt.artf_detector import Detector
except ImportError:
    print('\nWarning: missing torch!\n')

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize
from subt.tf_detector import CvDetector
from subt.artf_node import check_results, create_detector

g_streams = ["logimage.image", "logimage_rear.image"]
g_artf_names = ["survivor","backpack", "phone", "helmet", "rope", "fire_extinguisher", "drill", "vent", "cube"]

NAMES_AND_SCORES = {'backpack': 0.1,
                    'survivor': 0.1,
                    'phone': 0.1,
                    'rope': 0.1,
                    'helmet': 0.1,
                    'fire_extinguisher': 0.1,
                    'drill': 0.1,
                    'vent': 0.1,
                    'cube': 0.1,
                    'robot': 1.0,
                    'breadcrumb': 1.0,
                    'nothing': 1.0
                    }
MIN_SCORES = np.fromiter(NAMES_AND_SCORES.values(), float)


def proces_torch_result(result):
    result.sort(key=lambda item: item[2], reverse=True)
    score_torch = result[1][2]
    points = [[x, y] for x, y, c in result][:2]

    return score_torch, points


def manual_separation(data):
    ii = 0
    while True:
        if ii < 0:
            ii = 0
        if ii >= len(data):
            ii = len(data) - 1
        artf_name, score_t, score_cv, points, bbox, im_path, detection_type = data[ii]
        x, y, xw, yh = bbox
        im_path = im_path.rstrip("\r\n")
        img = cv2.imread(str(im_path), 1)
        assert img is not None
        cv2.rectangle(img, (x, y), (xw, yh), (0, 0, 255))
        cv2.putText(img, artf_name, (x, y), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255))
        for xc, yc in points:
            cv2.circle(img, (xc, yc), radius=4, color=(255, 255, 0), thickness=1)
        cv2.putText(img, str(detection_type), (10, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 0, 0))
        im_to_show = cv2.resize(img, (1280, 720))
        cv2.imshow("win", im_to_show)

        k = cv2.waitKey(0) & 0xFF
        if k == ord("n"):  # next img
            ii += 1
        elif k == ord("b"):  # back one img
            ii -= 1
        elif k == ord("t"):
            data[ii][6] = "true"  # true detection
        elif k == ord("f"):
            data[ii][6] = "false"  # false detection
        elif k == ord("d"):
            data[ii][6] = "None"  # do not use
        elif k == ord("q"):  # close and save
            break

    cv2.destroyAllWindows()

    return [[artf_name, score_t, score_cv, detection_type] for artf_name, score_t, score_cv, __, __, __, detection_type in data]


def log_eval(log_file):
    print(log_file)
    data_dir = log_file + ".d"
    try:
        os.makedirs(data_dir,exist_ok=False)  # do not overwrite existing data
    except FileExistsError:
        print("Directory already exist")
        return None
    detection_log = open(os.path.join(data_dir, "detection.log"), "w")
    confidence_thresholds = {
        'survivor': 0.5,
        'backpack': 0.5,
        'phone': 0.5,
        'helmet': 0.5,
        'rope': 0.5,
        'fire_extinguisher': 0.5,
        'drill': 0.5,
        'vent': 0.5,
        'cube': 0.5
    }

    detector = create_detector(confidence_thresholds)
    CvDetector().min_score = MIN_SCORES
    cv_detector = CvDetector().subt_detector

    for stream_name in g_streams:
        prefix = stream_name.split(".")[0]
        try:
            only_stream = lookup_stream_id(log_file, stream_name)
        except Exception as e:
            print(e)
            continue  # continue with another stream name
        with LogReader(log_file, only_stream_id=only_stream) as log:
            ii = 0
            for timestamp, stream_id, data_raw in log:
                buf_color = deserialize(data_raw)
                img = cv2.imdecode(np.frombuffer(buf_color, dtype=np.uint8), 1)

                result_cv = cv_detector(img)
                result_torch = detector(img)
                result = check_results(result_torch, result_cv)
                #print(result)

                im_file_name = prefix + "_im_%06d.jpg" % ii
                im_path = os.path.join(data_dir, im_file_name)
                for res in result:
                    print(res)
                    artf_name, res_t, res_cv = res
                    score_torch, points = proces_torch_result(res_t)
                    __, score_cv, bbox = res_cv

                    detection_log.write("%s;%s;%f;%f;%s;%s;%s\r\n" %(str(timestamp), artf_name, score_torch, score_cv,
                                                                     str(points), str(bbox), im_file_name))
                    detection_log.flush()
                    cv2.imwrite(im_path, img)

                ii += 1

    detection_log.close()


def plot_data(data, path, borders = None):
    for artf_name in g_artf_names:
        true_score_t = []
        true_score_cv = []
        false_score_t = []
        false_score_cv = []
        for artf, score_t, score_cv, detection_type in data:
            if detection_type == "true" and artf_name == artf:
                true_score_t.append(score_t)
                true_score_cv.append(score_cv)
            if detection_type == "false" and artf_name == artf:
                false_score_t.append(score_t)
                false_score_cv.append(score_cv)

        if true_score_t or false_score_t:
            fig = plt.figure(figsize=(7, 6))
            fig.subplots_adjust(left=0.1, right=0.8, bottom=0.1)

            ax1 = fig.add_subplot(111)
            ax1.plot(true_score_t, true_score_cv, "go", label="True")
            ax1.plot(false_score_t, false_score_cv, "ro", label="False")
            if borders:
                ax1.plot(borders[::2], borders[1::2], "b-o", label="Borders")
            ax1.grid()
            ax1.set_title(artf_name)
            ax1.set_xlabel("mdnet (-)")
            ax1.set_ylabel("cv detector (-)")
            ax1.legend(bbox_to_anchor=(1.05, 0.7))

            graph_name = datetime.datetime.now().strftime(artf_name + "_%y%m%d_%H%M%S")
            plt.savefig(os.path.join(path, graph_name), dpi=500)
            plt.show()


def plot_data_from_log(csv_file, artf_name, borders):
    data = []
    with open(csv_file) as data_csv:
        reader = csv.reader(data_csv, delimiter=';')
        for name, score_t, score_cv, detection_type in reader:
            if name == artf_name and detection_type in ["true", "false"]:
                data.append([name, float(score_t), float(score_cv), detection_type])
    data_dir, __ = os.path.split(csv_file)
    if borders:
        borders = [float(num) for num in borders.split()]
    plot_data(data, data_dir, borders = borders)


def separate_detections(path):
    # collect data with detection
    data_to_filter = []
    dir_list = sorted(os.listdir(path))
    for dir in dir_list:
        if not dir.endswith(".log.d"):
            continue
        for line in open(os.path.join(path, dir, "detection.log")):
            __, artf_name, score_t, score_cv, points, bbox, im_name = line.split(";")
            score_t = float(score_t)
            score_cv = float(score_cv)
            points = eval(points)
            bbox = eval(bbox)
            im_path = os.path.join(path, dir, im_name)
            data_to_filter.append([artf_name, score_t, score_cv, points, bbox, im_path, "true"])  # set detection_type to true as default

    data = manual_separation(data_to_filter)

    # save data for future analysis
    csv_name = datetime.datetime.now().strftime("detection_overview_%y%m%d_%H%M%S.csv")
    with open(os.path.join(path, csv_name), "w", newline='') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=";")
        for item in data:
            csv_writer.writerow(item)

    plot_data(data, path)


def eval_logs(path):
    if os.path.isdir(path):
        logs_list = sorted(os.listdir(path))
        for ii, log in enumerate(logs_list):
            if not log.endswith(".log"):
                continue
            if "rosout" in log or "server_console" in log:
                continue
            log_eval(os.path.join(path, log))
    else:
        log_eval(path)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfiles', help='Path to directory wit logfiles')
    parser.add_argument('--separate', help='Filter false detectio', action='store_true')
    parser.add_argument('--plot', help='Take artf name and plot data from file')
    parser.add_argument('--streams', help='List of image streams')
    parser.add_argument('--borders', help='borders points, e.g.: "0.6 1 0.9 0.65 0.95 0.2"')
    args = parser.parse_args()

    if args.streams:
        g_streams = args.streams.split()
    path = args.logfiles
    if args.separate:
        separate_detections(path)
    elif args.plot:
        plot_data_from_log(path, args.plot, args.borders)
    else:
        eval_logs(path)
