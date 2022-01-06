"""
  pyrealsense2 OSGAR wrapper

"""
import math
import logging
import threading
import numpy as np

from osgar.lib.quaternion import conjugate as quaternion_inv, euler_to_quaternion, multiply as quaternion_multiply, rotate_vector

g_logger = logging.getLogger(__name__)

try:
    import pyrealsense2 as rs
    try:
        prefix = rs.pyrealsense2.__version__[:4]
        if prefix in ('2.32', '2.33'):
            g_logger.error(f"RealSense version {rs.pyrealsense2.__version__} is broken in several ways!")
    except Exception as e:
        g_logger.error('pyrealsense2 version not found due to: ' + str(e))
except:
    g_logger.info('RealSense not installed!')
    from unittest.mock import MagicMock
    rs = MagicMock()

from osgar.node import Node
from osgar.lib import quaternion
from osgar.lib.serialize import serialize


# https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md#sensor-origin-and-coordinate-system

def t265_to_osgar_position(t265_position):
    x = -t265_position.z
    y = -t265_position.x
    z = t265_position.y
    return [x, y, z]


def t265_to_osgar_orientation(t265_orientation):
    x0 = t265_orientation.z
    y0 = -t265_orientation.x
    z0 = t265_orientation.y
    w0 = t265_orientation.w
    return [x0, y0, z0, w0]


class RealSense(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.device = config.get('device')

        if self.device == 'T200':
            bus.register('pose2d', 'pose3d', 'pose_raw', 'orientation')
            self.pose_subsample = config.get("pose_subsample", 20)
            self.tracking_sensor_pitch_quat_inv = quaternion_inv(
                    euler_to_quaternion(0, math.radians(config.get('tracking_sensor_pitch', 0)), 0))
            self.tracking_sensor_yaw_quat = euler_to_quaternion(
                math.radians(config.get('tracking_sensor_yaw', 0)), 0, 0)
        elif self.device in ['D400', 'L500']:
            self.depth_subsample = config.get("depth_subsample", 3)
            self.depth_rgb = config.get("depth_rgb", False)
            self.depth_infra = config.get("depth_infra", False)
            self.default_depth_resolution = config.get("depth_resolution", [640, 360])
            self.default_rgb_resolution = config.get("rgb_resolution", [640, 360])
            self.depth_fps = config.get("depth_fps", 30)
            self.disable_emitor = config.get("disable_emitor", False)
            self.camera_pose = config.get("camera_pose", [[0, 0, 0], [0, 0, 0, 1]])
            self.img_format = config.get("img_format", "*.jpeg")

            if self.depth_rgb or self.depth_infra:
                import cv2
                global cv2

            self.rgbd = config.get("rgbd", False)
            if self.rgbd:
                assert self.depth_rgb, self.depth_rgb
                bus.register('rgbd_raw:gz', 'infra')
            else:
                bus.register('depth:gz', 'color', 'infra')
        else:
            g_logger.warning("Device is not specified in the config!")

        self.serial_number = config.get('serial_number')
        if not self.serial_number:
            g_logger.info("Serial number is not set. Use rs-enumerate-devices tool to get camera information.")
            # https://github.com/IntelRealSense/librealsense/tree/master/tools/enumerate-devices
        self.verbose = config.get('verbose', False)
        self.pose_pipeline = None  # not initialized yet
        self.depth_pipeline = None
        self.finished = None

    def pose_callback(self, frame):
        try:
            pose = frame.as_pose_frame().get_pose_data()
            n = frame.get_frame_number()
            if n % self.pose_subsample != 0:
                return
            timestamp = frame.get_timestamp()
        except Exception as e:
            print(e)
            self.finished.set()
            return
        orientation = quaternion_multiply(
                self.tracking_sensor_pitch_quat_inv,
                t265_to_osgar_orientation(pose.rotation))
        self.publish('orientation', orientation)
        x, y, z = rotate_vector(t265_to_osgar_position(pose.translation),
                                self.tracking_sensor_yaw_quat)
        yaw = quaternion.heading(orientation)
        self.publish('pose2d', [int(x * 1000), int(y * 1000), int(math.degrees(yaw) * 100)])
        self.publish('pose3d', [[x, y, z], orientation])
        self.publish('pose_raw', [n, timestamp,
                             [pose.translation.x, pose.translation.y, pose.translation.z],
                             [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w],
                             [pose.velocity.x, pose.velocity.y, pose.velocity.z],
                             [pose.angular_velocity.x, pose.angular_velocity.y, pose.angular_velocity.z],
                             [pose.acceleration.x, pose.acceleration.y, pose.acceleration.z],
                             [pose.angular_acceleration.x, pose.angular_acceleration.y,
                              pose.angular_acceleration.z],
                             [pose.mapper_confidence, pose.tracker_confidence],
                             ])  # raw RealSense2 Pose data

    def depth_callback(self, frameset):
        try:
            assert frameset.is_frameset()
            if self.rgbd:
                frameset_as = self.align.process(frameset.as_frameset())
            else:
                frameset_as = frameset.as_frameset()
            depth_frame = frameset_as.get_depth_frame()

            n = depth_frame.get_frame_number()
            if n % self.depth_subsample != 0:
                return
            assert depth_frame.is_depth_frame()
            depth_image = np.asanyarray(depth_frame.as_depth_frame().get_data())

            if self.depth_rgb:
                color_frame = frameset_as.get_color_frame()
                assert color_frame.is_video_frame()
                color_image = np.asanyarray(color_frame.as_video_frame().get_data())
                __, data = cv2.imencode(self.img_format, color_image)

            if self.depth_infra:
                infra_frame = frameset_as.get_infrared_frame()
                assert infra_frame.is_video_frame()
                infra_image = np.asanyarray(infra_frame.as_video_frame().get_data())
                __, infra_data = cv2.imencode(self.img_format, infra_image)

        except Exception as e:
            print(e)
            self.finished.set()
            return

        if self.rgbd:
            self.publish('rgbd_raw', [None, self.camera_pose, data.tobytes(), serialize(depth_image)])  # None for robot pose
        else:
            self.publish('depth', depth_image)
            if self.depth_rgb:
                self.publish('color', data.tobytes())
        if self.depth_infra:
            self.publish('infra', infra_data.tobytes())


    def start(self):
        self.finished = threading.Event()
        ctx = rs.context()
        if self.device in ["D400", "L500"]:
            info_msg = "Enabling streams: depth"
            if self.depth_rgb:
                info_msg += ", depth_rgb"
                if self.rgbd:
                    info_msg += " (rgbd allowed)"
            if self.depth_infra:
                info_msg += ", depth_infra"
            g_logger.info(info_msg)

            self.depth_pipeline = rs.pipeline(ctx)
            depth_cfg = rs.config()
            if self.serial_number:
                depth_cfg.enable_device(self.serial_number)
            w, h = self.default_depth_resolution
            depth_cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, self.depth_fps)
            if self.depth_rgb:
                w, h = self.default_rgb_resolution
                depth_cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, self.depth_fps)
                if self.rgbd:
                    align_to = rs.stream.color
                    self.align = rs.align(align_to)
            if self.depth_infra:
                w, h = self.default_depth_resolution
                depth_cfg.enable_stream(rs.stream.infrared, w, h, rs.format.y8, self.depth_fps)
            if depth_cfg.can_resolve(self.depth_pipeline):
                profile = self.depth_pipeline.start(depth_cfg, self.depth_callback)
                if self.disable_emitor:
                    g_logger.info("Emitor disabled.")
                    device = profile.get_device()
                    depth_sensor = device.first_depth_sensor()  # there should be only one depth device
                    depth_sensor.set_option(rs.option.emitter_enabled, False)
            else:
                err_msg = "Can not resolve the configuration filters for depth device."
                if self.serial_number:
                    err_msg += " Serial number: %s" % self.serial_number
                g_logger.error(err_msg)
                self.depth_pipeline = None
                self.finished.set()

        elif self.device == "T200":
            g_logger.info("Enabling pose stream")
            self.pose_pipeline = rs.pipeline(ctx)
            pose_cfg = rs.config()
            if self.serial_number:
                pose_cfg.enable_device(self.serial_number)
            pose_cfg.enable_stream(rs.stream.pose)
            if pose_cfg.can_resolve(self.pose_pipeline):
                self.pose_pipeline.start(pose_cfg, self.pose_callback)
            else:
                err_msg = "Can not resolve the configuration filters for pose device."
                if self.serial_number:
                    err_msg += " Serial number: %s" %self.serial_number
                g_logger.error(err_msg)
                self.pose_pipeline = None
                self.finished.set()

        else:
            self.finished.set()

    def request_stop(self):
        if self.pose_pipeline is not None:
            self.pose_pipeline.stop()
        if self.depth_pipeline is not None:
            self.depth_pipeline.stop()
        super().request_stop()
        self.finished.set()

    def join(self, timeout_sec=None):
        self.finished.wait(timeout_sec)


class Multicam(Node):
    ''' Handles multiple RealSense cameras at once.

    This is useful when using a separate RealSense module for each camera doesn't work.
    '''
    def __init__(self, config, bus):
        super().__init__(config, bus)

        self.devices = config.get('devices')

        self.depth_subsample = config.get('depth_subsample', 3)
        self.depth_rgb = config.get('depth_rgb', False)
        self.depth_infra = config.get('depth_infra', False)
        self.pose_subsample = config.get('pose_subsample', 20)
        self.tracking_sensor_pitch_quat_inv = {}
        self.tracking_sensor_yaw_quat = {}
        self.camera_pose = config.get("camera_pose", [[0, 0, 0], [0, 0, 0, 1]])
        self.img_format = config.get("img_format", "*.jpeg")
        self.rgbd = config.get("rgbd", False)
        if self.rgbd:
            assert self.depth_rgb, self.depth_rgb

        streams = []
        for device in self.devices:
            device_type = device.get('type')
            serial_number = device.get('serial_number')
            if not serial_number:
                g_logger.info('Serial number is not set. Use rs-enumerate-devices tool to get camera information.')
                # https://github.com/IntelRealSense/librealsense/tree/master/tools/enumerate-devices
                continue
            name = device.get('name', serial_number)
            if device_type == 'T200':
                for stream in ['pose2d', 'pose3d', 'pose_raw', 'orientation']:
                    streams.append(f'{name}_{stream}')
                self.tracking_sensor_pitch_quat_inv[name] = quaternion_inv(
                        euler_to_quaternion(0, math.radians(device.get('pitch', 0)), 0))
                self.tracking_sensor_yaw_quat[name] = euler_to_quaternion(
                        math.radians(device.get('yaw', 0)), 0, 0)
            elif device_type in ['D400', 'L500']:
                if self.rgbd:
                    streams_to_register = ['rgbd_raw:gz', 'infra']
                else:
                    streams_to_register = ['depth:gz', 'color', 'infra']
                for stream in streams_to_register:
                    streams.append(f'{name}_{stream}')

                if self.depth_rgb or self.depth_infra:
                    import cv2
                    global cv2
            else:
                g_logger.warning('Device is not specified in the config!')

        bus.register(*streams)

        self.pipelines = []
        self.finished = None

        self.origins_lock = threading.Lock()
        self.origins = {}


    def pose_callback(self, frame):
        try:
            pose_frame = frame.as_pose_frame()
            profile = pose_frame.get_profile()
            with self.origins_lock:
                try:
                    name = self.origins[profile.unique_id()]
                except KeyError:
                    print('Pose callback call before registering the stream origin.')
                    return
            pose = pose_frame.get_pose_data()
            n = frame.get_frame_number()
            if n % self.pose_subsample != 0:
                return
            timestamp = frame.get_timestamp()
        except Exception as e:
            print(e)
            self.finished.set()
            return
        orientation = quaternion_multiply(
                self.tracking_sensor_pitch_quat_inv[name],
                t265_to_osgar_orientation(pose.rotation))
        self.publish(f'{name}_orientation', orientation)
        x, y, z = rotate_vector(t265_to_osgar_position(pose.translation),
                                self.tracking_sensor_yaw_quat[name])
        yaw = quaternion.heading(orientation)
        self.publish(f'{name}_pose2d', [int(x * 1000), int(y * 1000), int(math.degrees(yaw) * 100)])
        self.publish(f'{name}_pose3d', [[x, y, z], orientation])
        self.publish(f'{name}_pose_raw', [n, timestamp,
                             [pose.translation.x, pose.translation.y, pose.translation.z],
                             [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w],
                             [pose.velocity.x, pose.velocity.y, pose.velocity.z],
                             [pose.angular_velocity.x, pose.angular_velocity.y, pose.angular_velocity.z],
                             [pose.acceleration.x, pose.acceleration.y, pose.acceleration.z],
                             [pose.angular_acceleration.x, pose.angular_acceleration.y,
                              pose.angular_acceleration.z],
                             [pose.mapper_confidence, pose.tracker_confidence],
                             ])  # raw RealSense2 Pose data


    def depth_callback(self, frameset):
        try:
            assert frameset.is_frameset()
            if self.rgbd:
                frameset_as = self.align.process(frameset.as_frameset())
            else:
                frameset_as = frameset.as_frameset()
            depth_frame = frameset_as.get_depth_frame()

            profile = frameset.get_profile()
            with self.origins_lock:
                try:
                    name = self.origins[profile.unique_id()]
                except KeyError:
                    print('Depth callback call before registering the stream origin.')
                    return
            n = depth_frame.get_frame_number()
            if n % self.depth_subsample != 0:
                return
            assert depth_frame.is_depth_frame()
            depth_image = np.asanyarray(depth_frame.as_depth_frame().get_data())

            if self.depth_rgb:
                color_frame = frameset_as.get_color_frame()
                assert color_frame.is_video_frame()
                color_image = np.asanyarray(color_frame.as_video_frame().get_data())
                __, data = cv2.imencode(self.img_format, color_image)

            if self.depth_infra:
                infra_frame = frameset_as.get_infrared_frame()
                assert infra_frame.is_video_frame()
                infra_image = np.asanyarray(infra_frame.as_video_frame().get_data())
                __, infra_data = cv2.imencode(self.img_format, infra_image)

        except Exception as e:
            print(e)
            self.finished.set()
            return

        if self.rgbd:
            self.publish(f'{name}_rgbd_raw',
                         [None, self.camera_pose, data.tobytes(), serialize(depth_image)])  # None for robot pose
        else:
            self.publish(f'{name}_depth', depth_image)
            if self.depth_rgb:
                self.publish(f'{name}_color', data.tobytes())
        if self.depth_infra:
            self.publish(f'{name}_infra', infra_data.tobytes())


    def start(self):
        self.finished = threading.Event()
        ctx = rs.context()

        configs = []
        names = []
        callbacks = []
        for device in self.devices:
            device_type = device['type']
            serial_number = device['serial_number']
            name = device.get('name', serial_number)
            if device_type in ['D400', 'L500']:
                info_msg = f'Enabling streams: {name}_depth'
                if self.depth_rgb:
                    info_msg += f', {name}_color'
                    if self.rgbd:
                        info_msg += " (rgbd allowed)"
                if self.depth_infra:
                    info_msg += f', {name}_infra'
                g_logger.info(info_msg)

                depth_pipeline = rs.pipeline(ctx)
                depth_cfg = rs.config()
                depth_cfg.enable_device(serial_number)
                w, h = device.get('depth_resolution', [640, 360])
                depth_fps = device.get('depth_fps', 30)
                depth_cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, depth_fps)
                if self.depth_rgb:
                    w, h = device.get('rgb_resolution', [640, 360])
                    depth_cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, depth_fps)
                    if self.rgbd:
                        align_to = rs.stream.color
                        self.align = rs.align(align_to)
                if self.depth_infra:
                    w, h = device.get('depth_resolution', [640, 360])
                    depth_cfg.enable_stream(rs.stream.infrared, w, h, rs.format.y8, depth_fps)
                if depth_cfg.can_resolve(depth_pipeline):
                    self.pipelines.append(depth_pipeline)
                    configs.append(depth_cfg)
                    names.append(name)
                    callbacks.append(self.depth_callback)
                else:
                    err_msg = f'Can not resolve the configuration filters for depth device {name}.'
                    err_msg += ' Serial number: %s' % serial_number
                    g_logger.error(err_msg)
                    self.finished.set()

            elif device_type == 'T200':
                g_logger.info(f'Enabling {name} pose streams')
                pose_pipeline = rs.pipeline(ctx)
                pose_cfg = rs.config()
                pose_cfg.enable_device(serial_number)
                pose_cfg.enable_stream(rs.stream.pose)
                if pose_cfg.can_resolve(pose_pipeline):
                    self.pipelines.append(pose_pipeline)
                    configs.append(pose_cfg)
                    names.append(name)
                    callbacks.append(self.pose_callback)
                else:
                    err_msg = f'Can not resolve the configuration filters for pose device {name}.'
                    err_msg += ' Serial number: %s' % serial_number
                    g_logger.error(err_msg)
                    self.finished.set()

            else:
                self.finished.set()

        for (pipeline, cfg, name, callback) in zip(self.pipelines, configs, names, callbacks):
            pipeline_profile = pipeline.start(cfg, callback)
            with self.origins_lock:
                for stream in pipeline_profile.get_streams():
                    self.origins[stream.unique_id()] = name


    def request_stop(self):
        for pipeline in self.pipelines:
            pipeline.stop()
        super().request_stop()
        self.finished.set()


    def join(self, timeout_sec=None):
        self.finished.wait(timeout_sec)


# vim: expandtab sw=4 ts=4
