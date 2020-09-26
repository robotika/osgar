"""Artefacts detection via tensorflow"""

import os
import numpy as np
import cv2

# Directory with files saved_model.pb and saved_model.pbtxt
MODEL_DIR = "subt/tf_models"
# Files for CvDetector
PATH_TO_PB_GRAPH = "subt/tf_models/frozen_inference_graph.pb"
PATH_TO_CV_GRAPH = "subt/tf_models/cv_graph.pbtxt"

ARTF_NAME = np.array(["backpack", "survivor", "phone", "rope", "helmet", "robot"])
MIN_SCORES = np.array([0.2, 0.2, 0.2, 0.5, 0.3, 1.0])


class CvDetector:
    def __init__(self):
        if os.path.exists(PATH_TO_PB_GRAPH) and os.path.exists(PATH_TO_CV_GRAPH):
            graph_path = PATH_TO_PB_GRAPH
            cv_graph = PATH_TO_CV_GRAPH
        else:  # docker
            graph_path = os.path.join(os.path.dirname(__file__), '../../../frozen_inference_graph.pb')
            cv_graph = os.path.join(os.path.dirname(__file__), '../../../cv_graph.pbtxt')

        self.cvNet = cv2.dnn.readNetFromTensorflow(graph_path, cv_graph)

    def subt_detector(self, img):
        ret  = []
        detection = self.detect(img)
        if detection is not None:
            num_detections = detection['num_detections']
            for ii in range(num_detections):
                x, y, w, h = detection['bboxes'][ii]
                score = detection['scores'][ii]
                name = detection['classes_names'][ii]
                ret.append((name, [((x+w)//2, (y+h)//2, score)]))
        return ret

    def detect(self, img):
        rows = img.shape[0]
        cols = img.shape[1]
        # The size parameter should correspond with model
        # ssd_inception_v2
        self.cvNet.setInput(cv2.dnn.blobFromImage(img, size=(300, 300), swapRB=True, crop=False))
        # faster_rcnn_resnet50
#        self.cvNet.setInput(cv2.dnn.blobFromImage(img, size=(640, 320), swapRB=True, crop=False))
        cvOut = self.cvNet.forward()

        scores = cvOut[0, 0, :, 2]
        # ssd_inception_v2
        classes = cvOut[0, 0, :, 1]-1
        # faster_rcnn_resnet50
#        classes = cvOut[0, 0, :, 1]
        classes = classes.astype('int32')
        mask = MIN_SCORES[classes] < scores

        num_detections = np.count_nonzero(mask)
        if num_detections > 0:
            scores = scores[mask]
            classes = classes[mask]
            bboxes = cvOut[0, 0, :, 3:][mask]
            bboxes = bboxes*np.array([cols, rows, cols, rows])
            bboxes = bboxes.astype('int32')
            classes_names = ARTF_NAME[classes]

            return { 'bboxes': bboxes, 'classes_names': classes_names, 'scores': scores, 'num_detections': num_detections }

        return None


    def run_on_image(self, img):
        detection = self.detect(img)
        if detection:
            num_detections = detection['num_detections']
            for ii in range(num_detections):
                x, y, w, h = detection['bboxes'][ii]
                score = detection['scores'][ii]
                name = detection['classes_names'][ii]
                cv2.rectangle(img, (x, y), (w, h), (255, 0, 0), thickness=2)
                cv2.putText(img, "%s %0.3f " % (name, score), (min(x, 520), max(20, y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1,
                            cv2.LINE_AA)
        return img


class TfDetector:
    def __init__(self):
        import tensorflow as tf
        global tf
        self.model = tf.saved_model.load(MODEL_DIR)
        self.model = self.model.signatures['serving_default']
#        print(self.model.inputs)
        self.model.output_dtypes


    def detect(self, img):
        rows = img.shape[0]
        cols = img.shape[1]
        input_tensor = tf.convert_to_tensor(img)
        input_tensor = input_tensor[tf.newaxis, ...]
        raw_dict = self.model(input_tensor)

        scores = raw_dict['detection_scores'][0, :].numpy()
        classes = raw_dict['detection_classes'][0, :].numpy() - 1
        classes = classes.astype('int32')
        mask = MIN_SCORES[classes] < scores

        num_detections = np.count_nonzero(mask)
        if num_detections > 0:
            scores = scores[mask]
            classes = classes[mask]
            bboxes = raw_dict['detection_boxes'][0, :].numpy()[mask]
            bboxes = bboxes[:, [1, 0, 3, 2]]
            bboxes = bboxes*np.array([cols, rows, cols, rows])
            bboxes = bboxes.astype('int32')
            classes_names = ARTF_NAME[classes]

            return { 'bboxes': bboxes, 'classes_names': classes_names, 'scores': scores, 'num_detections': num_detections }

        return None


    def run_on_image(self, img):
        detection = self.detect(img)
        if detection:
            num_detections = detection['num_detections']
            for ii in range(num_detections):
                x, y, w, h = detection['bboxes'][ii]
                score = detection['scores'][ii]
                name = detection['classes_names'][ii]
                cv2.rectangle(img, (x, y), (w, h), (255, 0, 0), thickness=2)
                cv2.putText(img, "%s %0.3f " %(name, score), (min(x, 520), max(20, y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1,
                            cv2.LINE_AA)
        return img


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path', help='path to log file, image or directory wit images')
    parser.add_argument('--stream', help='image stream', default='camera.raw')
    parser.add_argument('--cam', help='test with notebook camera', action='store_true')
    parser.add_argument('--cv', help='run opencv detector', action='store_true')
    args = parser.parse_args()

    if args.cv:
        detector = CvDetector()
    else:
        detector = TfDetector()

    if args.cam:
        cam = cv2.VideoCapture(0)
        while True:
            __, img = cam.read()
            img = detector.run_on_image(img)
            cv2.imshow('img', img)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                cv2.destroyAllWindows()
                break
        cam.release()

    if args.path:
        path = args.path
        if os.path.isdir(path):
            im_file_list = os.listdir(path)
            for im_file in im_file_list:
                im_path = os.path.join(path, im_file)
                img = cv2.imread(im_path)
                if img is not None:
                    img = detector.run_on_image(img)
                    cv2.imshow('img', img)
                    k = cv2.waitKey(500) & 0xFF
                    if k == ord("q"):
                        cv2.destroyAllWindows()
                        break

        elif path.endswith(".log"):
            from osgar.logger import LogReader, lookup_stream_id
            from osgar.lib.serialize import deserialize
            stream_name = args.stream
            only_stream = lookup_stream_id(path, stream_name)
            with LogReader(path, only_stream_id=only_stream) as log:
                for timestamp, stream_id, data_raw in log:
                    buf_color = deserialize(data_raw)
                    img = cv2.imdecode(np.frombuffer(buf_color, dtype=np.uint8), 1)
                    img = detector.run_on_image(img)
                    cv2.imshow('img', img)
                    k = cv2.waitKey(200) & 0xFF
                    if k == ord("q"):
                        cv2.destroyAllWindows()
                        break

        else:
            img = cv2.imread(path)
            if img is not None:
                img = detector.run_on_image(img)
                cv2.imshow('img', img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            else:
                print("no image")
