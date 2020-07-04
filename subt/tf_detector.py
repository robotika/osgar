"""Artefacts detection via tensorflow"""

import os
import tensorflow as tf
import numpy as np
import cv2

# Directory with files saved_model.pb and saved_model.pbtxt
MODEL_DIR = "subt/tf_models"

class TfDetector:
    def __init__(self):
        self.model = tf.saved_model.load(MODEL_DIR)
        self.model = self.model.signatures['serving_default']
#        print(self.model.inputs)
        self.model.output_dtypes
        self.artf_names = np.array(["backpack", "survivor", "phone", "rope", "helmet"])
        self.min_scores = np.array([0.2, 0.2, 0.2, 0.2, 0.2])


    def detect(self, img):
        rows = img.shape[0]
        cols = img.shape[1]
        input_tensor = tf.convert_to_tensor(img)
        input_tensor = input_tensor[tf.newaxis, ...]
        raw_dict = self.model(input_tensor)

        scores = raw_dict['detection_scores'][0, :].numpy()
        classes = raw_dict['detection_classes'][0, :].numpy() - 1
        classes = classes.astype('int32')
        mask = self.min_scores[classes] < scores

        num_detections = np.count_nonzero(mask)
        if num_detections > 0:
            scores = scores[mask]
            classes = classes[mask]
            bboxes = raw_dict['detection_boxes'][0, :].numpy()[mask]
            bboxes = bboxes*np.array([rows, cols, rows, cols])
            bboxes = bboxes.astype('int32')
            classes_names = self.artf_names[classes]

            return { 'bboxes': bboxes, 'classes_names': classes_names, 'scores': scores, 'num_detections': num_detections }

        return None


    def run_on_image(self, img):
        detection = self.detect(img)
        if detection:
            num_detections = detection['num_detections']
            for ii in range(num_detections):
                y, x, h, w = detection['bboxes'][ii]
                score = detection['scores'][ii]
                name = detection['classes_names'][ii]
                cv2.rectangle(img, (x, y), (w, h), (255, 0, 0), thickness=2)
                cv2.putText(img, "%s %0.3f " %(name, score), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1,
                            cv2.LINE_AA)
        return img


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path', help='path to log file, image or directory wit images')
    parser.add_argument('--stream', help='image stream', default='camera.raw')
    parser.add_argument('--cam', help='test with notebook camera', action='store_true')
    args = parser.parse_args()

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
