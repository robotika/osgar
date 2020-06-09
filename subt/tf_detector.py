import sys
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
        self.min_scores = np.array([0.2, 0.8, 0.2, 0.8, 0.2])


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

#        print("scores: ", scores)
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


    def show_result(self, img, detection, ret_img = False):
        if detection:
            num_detections = detection['num_detections']
            for ii in range(num_detections):
                y, x, h, w = detection['bboxes'][ii]
                score = detection['scores'][ii]
                name = detection['classes_names'][ii]
                cv2.rectangle(img, (x, y), (w, h), (255, 0, 0), thickness=2)
                cv2.putText(img, name+' '+str(score), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
            if not ret_img:
                cv2.imshow('img', img)
                cv2.waitKey(1000)
        elif not ret_img:
            cv2.imshow('img', img)
            cv2.waitKey(1)

        return img


if __name__ == "__main__":
    detector = TfDetector()
    if sys.argv[1] == "cam":
        cam = cv2.VideoCapture(0)
        while True:
            __, img = cam.read()
            bbox = detector.detect(img)
            print(bbox)
            detector.show_result(img, bbox)

    path = sys.argv[1]
    if os.path.isdir(path):
        im_file_list = os.listdir(path)
        for im_file in im_file_list:
            im_path = os.path.join(path, im_file)
            img = cv2.imread(im_path)
            if img is not None:
                detection = detector.detect(img)
                if detection:
                    print(detection)
                detector.show_result(img, detection)

    elif path.endswith(".log"):
        from osgar.logger import LogReader, lookup_stream_id
        from osgar.lib.serialize import deserialize
        import time
        proc_time = []

        if len(sys.argv) == 3:
            stream_name = sys.argv[2]
        else:
            stream_name = "camera.raw"

        only_stream = lookup_stream_id(path, stream_name)
        with LogReader(path, only_stream_id=only_stream) as log:
            ii = 0
            for timestamp, stream_id, data_raw in log:
                buf_color = deserialize(data_raw)
                img = cv2.imdecode(np.fromstring(buf_color, dtype=np.uint8), 1)
                t0 = time.time()
                detection = detector.detect(img)
                proc_time.append(time.time() - t0)
                if detection:
                    print(detection)
#                    cv2.imwrite("tmp/im%05d.png" %ii, img)
                detector.show_result(img, detection)
                ii += 1
            print("Average processing time: ", np.mean(proc_time))

    else:
        img = cv2.imread(path)
        if img is not None:
            detection = detector.detect(img)
            if detection is not None:
                detector.show_result(img, detection)
        else:
            print("no image")
