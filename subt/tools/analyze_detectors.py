"""evaluate detectors"""
import os
import numpy as np
import cv2

try:
    import torch
    import subt.artf_model
    from subt.artf_detector import Detector
except ImportError:
    print('\nWarning: missing torch!\n')

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize
from subt.tf_detector import CvDetector
from subt.artf_node import check_results

g_streams = ["logimage.image", "logimage_rear.image"]


def take_third(item):
    return item[2]


def proces_torch_result(result):
    result.sort(key=take_third, reverse=True)
    score_torch = result[1][2]
    points = [[x, y] for x, y, c in result][:2]

    return score_torch, points


def create_detector():
    model = os.path.join(os.path.dirname(__file__), '../../../mdnet5.128.128.13.4.elu.pth')
    confidence_thresholds = {
        'survivor': 0.5,
        'backpack': 0.5,
        'phone': 0.5,
        'helmet': 0.5,
        'rope': 0.5,
        'fire_extinguisher': 0.5,
        'drill': 0.5,
        'vent': 0.5,
        'cube' : 0.5
    }
    max_gap = 16
    min_group_size = 2

    use_cuda = torch.cuda.is_available()
    device = torch.device("cuda" if use_cuda else "cpu")
    print('Using:', device)
    model, categories = subt.artf_model.load_model(model, device)
    return Detector(model, confidence_thresholds, categories, device,
                    max_gap, min_group_size)


def log_eval(log_file):
    data_dir = log_file + ".d"
    os.makedirs(data_dir,exist_ok=True)
    detection_log = open(os.path.join(data_dir, "detection.log"), "w")

    detector = create_detector()
    CvDetector().min_score = [0.1,]*12  # there are also robot, breadcrumb and nothing objects
    cv_detector = CvDetector().subt_detector

    for stream_name in g_streams:
        prefix = stream_name.split(".")[0]
        only_stream = lookup_stream_id(log_file, stream_name)
        with LogReader(log_file, only_stream_id=only_stream) as log:
            ii = 0
            for timestamp, stream_id, data_raw in log:
                buf_color = deserialize(data_raw)
                img = cv2.imdecode(np.frombuffer(buf_color, dtype=np.uint8), 1)

                result_cv = cv_detector(img)
                result_torch = detector(img)
                result = check_results(result_torch, result_cv)
                #print(result)

                im_file_name = os.path.join(data_dir, prefix + "_im_%05d.jpg" % ii)
                for res in result:
                    print(res)
                    artf_name, res_t, res_cv = res
                    score_torch, points = proces_torch_result(res_t)
                    __, score_cv, bbox = res_cv

                    detection_log.write("%s;%f;%f;%s;%s;%s\r\n" %(artf_name, score_torch, score_cv, str(points), str(bbox), im_file_name))
                    detection_log.flush()
                    cv2.imwrite(im_file_name, img)

                ii += 1

    detection_log.close()


def plot_data(path):
    pass


def filter_detections(path):
    pass


def eval_logs(path):
    if os.path.isdir(path):
        logs_list = sorted(os.listdir(path))
        for ii, log in enumerate(logs_list):
            if not log.endswith("log"):
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
    parser.add_argument('--filter', help='Filter false detection', action='store_true')
    parser.add_argument('--plot', help='Plot score', action='store_true')
    args = parser.parse_args()

    path = args.logfiles
    if args.filter:
        filter_detections(path)
    elif args.plot:
        plot_data(path)
    else:
        eval_logs(path)

