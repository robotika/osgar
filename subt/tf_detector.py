import sys
import tensorflow as tf
import numpy as np
import cv2


MODEL_DIR = "tf_models"
PATH_TO_LABELS = "tf_models/subt_label_map.pbtxt"


class Tf_detector:
    def __init__(self):
        model = tf.saved_model.load(MODEL_DIR)
        self.model = model.signatures['serving_default']
#        print(self.model.inputs)
        self.model.output_dtypes

    def detect(self, img):
        rows = img.shape[0]
        cols = img.shape[1]
#        image = np.asarray(img)
        input_tensor = tf.convert_to_tensor(img)
        input_tensor = input_tensor[tf.newaxis, ...]
        raw_dict = self.model(input_tensor)
        print(raw_dict)

        num_detections = int(raw_dict['num_detections'])
        print(num_detections)
#        output_dict = {key: value[0, :num_detections].numpy()
#                       for key, value in output_dict.items()}
        bboxes = raw_dict['detection_boxes'][0,:num_detections, :]
        print(bboxes)
        classes = raw_dict['detection_classes'][:num_detections]
        scores = raw_dict['detection_scores'][:num_detections]
        sys.exit()

        print(output_dict)
#TODO...
        output_dict['num_detections'] = num_detections
        print(output_dict)
        output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)
        print(output_dict)

        if len(output_dict["detection_boxes"]) > 0:
            bbox = output_dict["detection_boxes"][0]
            x = bbox[1] * cols
            y = bbox[0] * rows
            w = bbox[3] * cols
            h = bbox[2] * rows
            return [x, y, w, h]

        return None

    def show_result(self, img, bbox):
        x, y, w, h = bbox
        cv2.rectangle(img, (int(x), int(y)), (int(w), int(h)), (125, 255, 51), thickness=2)
        cv2.imshow('img', img)
        cv2.waitKey(100)

if __name__ == "__main__":
    detector = Tf_detector()
    if sys.argv[1] == "cam":
        cam = cv2.VideoCapture(0)
        while True:
            __, img = cam.read()
            bbox = detector.detect(img)
            print("frame")
            if bbox is not None:
                detector.show_result(img, bbox)

    img = cv2.imread(sys.argv[1])
    if img is not None:
        bbox = detector.detect(img)
        detector.show_result(img, bbox)
    else:
        print("no image")