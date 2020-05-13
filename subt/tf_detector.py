import sys
import tensorflow as tf
import numpy as np
import cv2


MODEL_DIR = "tf_models"

class Tf_detector:
    def __init__(self):
        model = tf.saved_model.load(MODEL_DIR)
        self.model = model.signatures['serving_default']
#        print(self.model.inputs)
        self.model.output_dtypes
        self.artf_names = np.array(["backpack", "survivor", "phone"])


    def detect(self, img):
        rows = img.shape[0]
        cols = img.shape[1]
#        image = np.asarray(img)
        input_tensor = tf.convert_to_tensor(img)
        input_tensor = input_tensor[tf.newaxis, ...]
        raw_dict = self.model(input_tensor)

        num_detections = int(raw_dict['num_detections'])
        if num_detections > 0:
            print(num_detections)
            bboxes = raw_dict['detection_boxes'][0,:num_detections].numpy()
            bboxes = bboxes*np.array([rows, cols, rows, cols])
            bboxes = bboxes.astype('int32')
            classes = raw_dict['detection_classes'][0, :num_detections].numpy() - 1
            classes = classes.astype('int32')
            scores = raw_dict['detection_scores'][0, :num_detections].numpy()
            classes_names = self.artf_names[classes]

            return { 'bboxes': bboxes, 'classes_names': classes_names, 'scores': scores, 'num_detections': num_detections }

        return None

    def show_result(self, img, detection):
        num_detections = detection['num_detections']
        for ii in range(num_detections):
            y, x, h, w = detection['bboxes'][ii]
            score = detection['scores'][ii]
            name = detection['classes_names'][ii]
            cv2.rectangle(img, (x, y), (w, h), (255, 0, 0), thickness=2)
            print(x, y)
            cv2.putText(img, name+' '+str(score), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

        cv2.imshow('img', img)
        cv2.waitKey(0)

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
        detection = detector.detect(img)
        if detection is not None:
            detector.show_result(img, detection)
    else:
        print("no image")