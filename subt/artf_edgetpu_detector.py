import numpy as np
from io import BytesIO
from PIL import Image

from pycoral.utils import edgetpu
from pycoral.adapters import common as coral_common
from pycoral.adapters import detect as coral_detection

from osgar.lib.depth import compress, decompress
from osgar.lib.quaternion import transform
from osgar.node import Node
from subt.artf_utils import NAME2IGN

class Detector(Node):
    def __init__(self, config, bus, verbose=False):
        super().__init__(config, bus)
        bus.register("localized_artf", "debug_rgbd")

        self.verbose = verbose

        model_path = config.get('model_path', 'subt/models/system/edgetpu.0/model_edgetpu.tflite')
        self.interpreter = edgetpu.make_interpreter(model_path)
        self.interpreter.allocate_tensors()

        self.thresholds = config.get('thresholds', {
            'backpack': 0.84,
            'survivor': 0.95,
            'phone': 1000,  # Disabled.
            'rope': 0.85,
            'helmet': 0.95,
            'fire_extinguisher': 0.85,
            'drill': 0.9,
            'vent': 0.95,
            'cube': 1000  # Disabled.
            })
        self.categories = dict(enumerate(self.thresholds.keys()))
        self.min_threshold = min(self.thresholds.values())

        self.min_depth = config.get('min_depth', 0.1)
        self.max_depth = config.get('max_depth', 10.0)
        self.min_valid_depth_pixels = config.get('min_valid_depth_pixels', 4)

        self.camera_params = config['camera']

        self.input_size = coral_common.input_size(self.interpreter)


    def update(self):
        timestamp, channel, data  = self.bus.listen()
        robot_pose, camera_pose, img_data, compressed_depth = data
        img = Image.open(BytesIO(img_data)).convert('RGB')

        input_img = img.resize(self.input_size, Image.ANTIALIAS)
        scale = input_img.width / float(img.width), input_img.height / float(img.height)
        coral_common.set_input(self.interpreter, input_img)
        self.interpreter.invoke()
        detections = coral_detection.get_objects(self.interpreter, self.min_threshold, image_scale=scale)

        if not detections:
            return

        depth = decompress(compressed_depth)
        camera_params = self.camera_params[channel]

        for detection in detections:
            category = self.categories.get(detection.id)
            if category is None:
                # This is one of the unsupported categories, such as "robot' or 'nothing'.
                continue
            threshold = self.thresholds[category]
            if detection.score >= threshold:
                xmin = np.clip(detection.bbox.xmin, 0, img.width)
                xmax = np.clip(detection.bbox.xmax, 0, img.width)
                ymin = np.clip(detection.bbox.ymin, 0, img.height)
                ymax = np.clip(detection.bbox.ymax, 0, img.height)
                patch = [v for v in depth[ymin:ymax, xmin:xmax].reshape((-1))
                         if v >= self.min_depth and v < self.max_depth]
                if len(patch) < self.min_valid_depth_pixels:
                    continue
                d = np.median(patch)
                u = (xmin + xmax) / 2
                v = (ymin + ymax) / 2

                # Location of the artifact relative to the camera.
                x = d
                y = d * (camera_params['cy'] - u) / camera_params['fx']
                z = d * (camera_params['cx'] - v) / camera_params['fy']

                # Coordinate of the artifact relative to the robot.
                robot_rel = transform([x, y, z], camera_pose)
                # Global coordinate of the artifact.
                world_xyz = transform(robot_rel, robot_pose)

                ign_name = NAME2IGN[category]

                if self.verbose:
                    print(ign_name, world_xyz, detection.score)
                self.publish('localized_artf', [ign_name, world_xyz])
                # Making sure the output depth is compressed. Input depth may or may not be.
                self.publish('debug_rgbd', [robot_pose, camera_pose, img_data, compress(depth)])
