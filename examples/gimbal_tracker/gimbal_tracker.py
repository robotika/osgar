"""
zdroj videa (OSGAR "opencv" Node)
- Detector - vstup snimky, vyber nejnovejsi, detekce, publikuj cas prichoziho snimku a pole bboxu
- Servo - dostavej par integeru odpovidajici raw PWM
- Controller, sbira info o bboxech, pripadne si trackuje ocekavanou polohu serv v case a vysila prika
"""

from osgar.node import Node

import cv2
import numpy as np
import time

import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/

from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils




class Detector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('bbox')
        
        # Initialize the object detection model
        model='/home/pi/examples/lite/examples/object_detection/raspberry_pi/efficientdet_lite0.tflite'
        enable_edgetpu=False
        num_threads=4
        base_options = core.BaseOptions(file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
        detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.3)
        options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
        self.detector = vision.ObjectDetector.create_from_options(options)
        
        #---- test
        # Variables to calculate FPS
        self.counter, self.fps = 0, 0
        self.start_time = time.time()
        #---- test
        
    def on_raw(self, data):
        #pass
        image = cv2.imdecode(np.fromstring(data, dtype=np.uint8), 1)
        image = cv2.flip(image, 1)
        
        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Create a TensorImage object from the RGB image.
        input_tensor = vision.TensorImage.create_from_array(rgb_image)

        # Run object detection estimation using the model.
        detection_result = self.detector.detect(input_tensor)
        bbox=[]
        for detection in detection_result.detections:
            if detection.categories[0].category_name=='person':
                #print(detection.bounding_box)

                bb_center_x=detection.bounding_box.origin_x+detection.bounding_box.width/2
                bb_center_y=detection.bounding_box.origin_y+detection.bounding_box.height/2
                cv2.rectangle(image, (int(bb_center_x-10), int(bb_center_y-10)), (int(bb_center_x+10), int(bb_center_y+10)), (255, 0, 0))
                
                bbox.append([bb_center_x, bb_center_y])
        
        self.publish('bbox', bbox)
        
        #---- test
        # Draw keypoints and edges on input image
        self.counter += 1
        image = utils.visualize(image, detection_result)
        
        # Calculate the FPS

        fps_avg_frame_count = 10
        
        if self.counter % fps_avg_frame_count == 0:
          end_time = time.time()
          self.fps = fps_avg_frame_count / (end_time - self.start_time)
          self.start_time = time.time()

        # Show the FPS
        fps_text = 'FPS = {:.1f}'.format(self.fps)
        text_location = (24, 20) #left_margin, row_size
        cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                    1, (0, 0, 255), 1) # font size, text_color, font_thickness
    
        cv2.imshow('object_detector', image)
        
        cv2.waitKey(1)
        #---- test
        

        
    def on_position(self, data):
        pass



class Controller(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_position')
        self.unsec_det=0
        self.angle_x=90
        self.angle_y=90

    def on_position(self, data):
        pass

    
    def on_bbox(self, data):
        
        P=0.01
        im_width=640
        im_height=480

        
                
        if len(data) > 0:
            #print(data)
            bb_center_x=data[0][0] # beru prvni detekci .. tady by se mel resit tracking
            bb_center_y=data[0][1] 
            
            
            
            if 0 < bb_center_x < im_width and 0 < bb_center_y < im_height:
                self.angle_x=self.angle_x+P*(bb_center_x-im_width/2) # 62.2/640 P by melo byt 0.1
                self.angle_y=self.angle_y-P*(bb_center_y-im_height/2) # 48.8/480 P by melo byt 0.1
                
                if 10 < self.angle_x < 170 and 10 < self.angle_y < 170:
                    
                    self.unsec_det=0
                    person_detected=1
                
                    self.publish('desired_position', [self.angle_x,self.angle_y])
                    #time.sleep(1)
                else:
                    msg='angle out of range = reset tracking'
                    self.angle_x=90
                    self.angle_y=90
                    self.publish('desired_position', [self.angle_x,self.angle_y])
                                 
                
        else: 
            self.unsec_det=self.unsec_det+1
            
        if self.unsec_det==5:
            msg='person not detected = reset tracking'
            angle_x=90
            angle_y=90
            self.publish('desired_position', [self.angle_x,self.angle_y])
            self.unsec_det=0
        elif self.unsec_det==0:
            msg='person detected ' + "%d" % self.angle_x + " " + "%d" % self.angle_y
        else:
            msg='person not detected ' + "%d" % self.unsec_det
            
        #print(msg)
        



class Servo(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('position')

        #inicialialization and setting servo on default position !! no logic to slow down movement ..
        
        self.position_x = 90
        self.position_y = 90
        
        self.angle_x = 90
        self.angle_y = 90

        self.pca = ServoKit(channels=16)

        self.pca.servo[0].set_pulse_width_range(500 , 2500)
        self.pca.servo[1].set_pulse_width_range(500 , 2500)
        
        self.pca.servo[0].angle = self.angle_x
        self.pca.servo[1].angle = self.angle_y
        
        
    def on_tick(self, data):

        if self.position_x<round(self.angle_x):
                self.position_x += 1
            
        if self.position_y>round(self.angle_y):
                self.position_y -= 1
        
        self.pca.servo[0].angle = self.position_x
        self.pca.servo[1].angle = self.position_y
        self.publish('position', [self.position_x, self.position_y] )
        #print(self.position)

        
    def on_desired_position(self, data):
        self.angle_x=data[0]
        self.angle_y=data[1]
        print(data)

