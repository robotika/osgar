"""
  Draw laser scans
"""
import math
import io
import pathlib
from ast import literal_eval
from datetime import timedelta
from collections import defaultdict

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "1"
import pygame
from pygame.locals import *
import cv2  # for video output
import numpy as np  # faster depth data processing
from importlib import import_module

from osgar.logger import LogIndexedReader, lookup_stream_names, LogReader
from osgar.lib.serialize import deserialize
from osgar.lib.config import get_class_by_name
from osgar.lib import quaternion
from osgar.lib.depth import depth2danger, DepthParams, decompress as decompress_depth


WINDOW_SIZE = 1600, 1000  # controlled by --window-size
TAIL_MIN_STEP = 0.1  # in meters
HISTORY_SIZE = 100

MAX_SCAN_LIMIT = 10000  # set by --lidar-limit

g_scale = 30
g_rotation_offset_rad = 0.0  # set by --rotation (deg)
g_lidar_fov_deg = 270  # controlled by --deg (deg)

# depth data for ROBOTIKA_X2_SENSOR_CONFIG_1 (640 x 360)
g_depth_params = DepthParams()
g_prefix = "saveX"
g_log_config = None  # original log config file

CENTER_AXLE_DISTANCE = 0.348  # K2 distance specific


def scr(x, y):
    global g_scale
    return WINDOW_SIZE[0]//2 + round(g_scale*x), WINDOW_SIZE[1]//2 - round(g_scale*y)


def scan2xy(pose, scan):
    X, Y, heading = pose
    pts = []
    for i, i_dist in enumerate(scan):
        if i_dist == 0 or i_dist >= MAX_SCAN_LIMIT:
            continue
        angle = math.radians(g_lidar_fov_deg * (i / len(scan)) - g_lidar_fov_deg/2) + heading
        dist = i_dist/1000.0
        x, y = dist * math.cos(angle), dist * math.sin(angle)
        pts.append((X + x, Y + y))
    return pts


def filter_pts(grid, pts):
    # filter out duplicity points - keep older
    ret = []
    for x, y in pts:
        k = int(2*x), int(2*y)
        if grid[k] < 10:
            ret.append((x, y))
            grid[k] += 1
    return ret


def draw_scan(foreground, pose, scan, color, joint=None):
    __, __, heading = pose
    X, Y = 0, 0  # front lidar is in the image center

    if joint is not None:
        # correct for joint angle
        dist = CENTER_AXLE_DISTANCE
        heading += math.pi
        dx, dy = dist * math.cos(heading), dist * math.sin(heading)
        X += dx
        Y += dy
        for j in joint:
            heading -= math.radians(j / 100.0)
            dx, dy = dist * math.cos(heading), dist * math.sin(heading)
            X += dx
            Y += dy

    for i, i_dist in enumerate(scan):
        if i_dist == 0 or i_dist >= MAX_SCAN_LIMIT:
            continue
        angle = math.radians(g_lidar_fov_deg * (i / len(scan)) - g_lidar_fov_deg/2) + heading
        dist = i_dist/1000.0
        x, y = dist * math.cos(angle), dist * math.sin(angle)
        pygame.draw.circle(foreground, color, scr(X + x, Y + y), 3)


def draw_scan_up_down(foreground, pose, lidar_up, lidar_down, color):
    __, __, heading = pose

    if lidar_up is not None:
        pygame.draw.circle(foreground, color, scr(0, lidar_up[0]), 3)

    if lidar_down is not None:
        pygame.draw.line(foreground, color, scr(0, 0), scr(0, -lidar_down[0]), 2)
        pygame.draw.circle(foreground, color, scr(0, -lidar_down[0]), 3)


def draw(foreground, pose, scan, poses=[], image=None, bbox=None, callback=None, acc_pts=None):
    foreground.lock()
    color = (0, 255, 0)
    X, Y, heading = pose
    for i, i_dist in enumerate(scan):
        if i_dist == 0 or i_dist >= MAX_SCAN_LIMIT:
            continue
        angle = math.radians(g_lidar_fov_deg * (i / len(scan)) - g_lidar_fov_deg/2) + heading
        dist = i_dist/1000.0
        x, y = dist * math.cos(angle), dist * math.sin(angle)
        pygame.draw.circle(foreground, color, scr(x, y), 3)

    # draw scale
    meter_px = scr(1, 0)[0] - scr(0, 0)[0]
    height = WINDOW_SIZE[1]
    pygame.draw.line(foreground, (255, 0, 0), (20, height - 20), (20 + meter_px, height - 20), 3)
    pygame.draw.line(foreground, (255, 0, 0), (20, height - 10), (20, height - 30), 1)
    pygame.draw.line(foreground, (255, 0, 0), (20 + meter_px, height - 10), (20 + meter_px, height - 30), 1)

    # draw LIDAR pose
    pygame.draw.circle(foreground, (147, 20, 255), scr(0, 0), 5)
    pygame.draw.line(foreground, (147, 20, 255), scr(0, 0), scr(0.5*math.cos(heading), 0.5*math.sin(heading)), 1)

    # draw trace of poses
    for x, y, __ in poses:
        pygame.draw.circle(foreground, (255, 128, 0), scr(x - X, y - Y), 2)

    # draw accumulated points
    if acc_pts is not None:
        for x, y in acc_pts:
            pygame.draw.circle(foreground, (0, 127, 0), scr(x - X, y - Y), 2)

    foreground.unlock()

    if image is not None:
        width, height = image.get_size()
        if width > WINDOW_SIZE[0] or height > WINDOW_SIZE[1]:
            width, height = (512, 384)
        cameraView = pygame.transform.scale(image, (width, height))

        foreground.blit(cameraView, (0, 0))

        def frameNorm(w, h, bbox):
            normVals = np.full(len(bbox), w)
            normVals[::2] = h
            return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

        for frame_detections in bbox:
            for detection in frame_detections:
                if len(detection) == 0:
                    continue  # empty bbox is allowed now
                # old format (b=detection)
                #assert len(b) > 5, b
                #name, x, y, width, height = b[:5]
                # new (temporary?) format
                w, h = image.get_size()
                # stream name
                image_size = g_log_config['robot']['modules']['oak']['init']['nn_config']['input_size']
                assert image_size in ['640x352', '640x640', '416x416'], image_size
                nn_w, nn_h = [int(v) for v in image_size.split('x')]
                if nn_h == nn_w:
                    # squared model
                    a, b, c, d = frameNorm(h, h, detection[2]).tolist()
                    name, x, y, width, height = detection[0], a + (w - h) / 2, b, c - a, d - b
                else:
                    a, b, c, d = frameNorm(h, w, detection[2]).tolist()
                    name, x, y, width, height = detection[0],  a, b, c - a, d - b
                color = (0, 255, 0)
                rect = pygame.Rect(x, y, width, height)
                pygame.draw.rect(image, color, rect, 4)
                font = pygame.font.SysFont("arial", 60)
                text_surface = font.render(name[:3], True, color)
                text_rect = text_surface.get_rect()
                text_rect.center = rect.center
                image.blit(text_surface, text_rect)

    if callback is not None:
        debug_poly = []
        callback(pose, scan, debug_poly)
        for poly in debug_poly:
            prev = None
            pts = [scr(x, y) for x, y in poly]
            for a, b in zip(pts[:-1], pts[1:]):
                pygame.draw.line(foreground, (200, 200, 0), a, b, 1)


def draw_robot(foreground, pose, joint):

    if joint is not None:
        color = (255, 0, 0)
        __, __, heading = pose

        # first draw line from lidar to joint
        angle = heading + math.pi
        dist = CENTER_AXLE_DISTANCE
        x, y = dist*math.cos(angle), dist*math.sin(angle)
        pygame.draw.line(foreground, color, scr(0, 0), scr(x, y), 3)

        # joint itself
        pygame.draw.circle(foreground, color, scr(x, y), 3)

        # rear part - now K2 specific
        angle -= math.radians(joint[0]/100.0)
        dx, dy = dist*math.cos(angle), dist*math.sin(angle)
        pygame.draw.line(foreground, color, scr(x, y), scr(x + dx, y + dy), 3)

        if len(joint) > 1:
            # K3 specific
            pygame.draw.circle(foreground, color, scr(x + dx, y + dy), 3)
            angle -= math.radians(joint[1]/100.0)
            dx2, dy2 = dist*math.cos(angle), dist*math.sin(angle)
            pygame.draw.line(foreground, color, scr(x + dx, y + dy), scr(x + dx + dx2, y + dy + dy2), 3)


g_depth = None
g_danger_binary_image = False

def depth_map(depth):
    MAX_RANGE = 10.0
    depth = (255 * depth / MAX_RANGE).astype(np.uint8)
    return pygame.image.frombuffer(
            cv2.cvtColor(
                cv2.applyColorMap(depth, cv2.COLORMAP_JET),
                cv2.COLOR_BGR2RGB).tobytes(),
            depth.shape[1::-1], "RGB")

def get_image(data):
    """Extract JPEG or RGBD depth image"""
    global g_depth
    # https://stackoverflow.com/questions/12569452/how-to-identify-numpy-types-in-python
    if isinstance(data, np.ndarray):
        # https://www.learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/
        if g_danger_binary_image:
            img = np.array(depth2danger(data / 1000, g_depth_params) * 255, dtype=np.uint8)
            im_color = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        else:
            img = np.array(np.minimum(255*40, data)/40, dtype=np.uint8)
            im_color = cv2.applyColorMap(img, cv2.COLORMAP_JET)

        # https://stackoverflow.com/questions/19306211/opencv-cv2-image-to-pygame-image
        image = pygame.image.frombuffer(im_color.tobytes(), im_color.shape[1::-1], "RGB")
        g_depth = data
    elif isinstance(data, tuple):
        img_data, depth_data = data
        image = pygame.image.load(io.BytesIO(img_data), 'JPG').convert()
        g_depth = decompress_depth(depth_data)
    elif isinstance(data, list):
        # image stereo artefact localization
        # expects localized pair of images [camera_name, [robot_pose, camera_pose, image], [robot_pose, camera_pose, image]]
        assert len(data) == 3, len(data)
        image = pygame.image.load(io.BytesIO(data[1][2]), 'JPG').convert()
    elif data is not None:
        try:
            image = pygame.image.load(io.BytesIO(data), 'JPG').convert()
        except pygame.error:
            # try H264 via OpenCV
            is_h264 = data.startswith(bytes.fromhex('00000001 0950')) or data.startswith(bytes.fromhex('00000001 0930'))
            is_h265 = data.startswith(bytes.fromhex('00000001 460150')) or data.startswith(bytes.fromhex('00000001 460130'))
            assert is_h264 or is_h265, data[:20].hex()
            if data.startswith(bytes.fromhex('00000001 0950')) or data.startswith(bytes.fromhex('00000001 460150')):
                # I - key frame
                with open('tmp.h264', 'wb') as f:
                    f.write(data)
            elif data.startswith(bytes.fromhex('00000001 0930')) or data.startswith(bytes.fromhex('00000001 460130')):
                # P-frame}
                with open('tmp.h264', 'ab') as f:
                    f.write(data)
            else:
                assert 0, f'Unexpected data {data[:20].hex()}'
            cap = cv2.VideoCapture('tmp.h264')
            image = None
            ret = True
            while ret:
                ret, frame = cap.read()
                if ret:
                    image = pygame.image.frombuffer(frame.tobytes(), frame.shape[1::-1], "BGR")
            cap.release()
    else:
        image = None
    return image, (None if g_depth is None else depth_map(g_depth))


def pygame_to_numpy_image(pygame_img):
    view = pygame.surfarray.array3d(pygame_img)
    view = view.transpose([1, 0, 2])
    np_img = cv2.cvtColor(view, cv2.COLOR_RGB2BGR)
    return np_img


def numpy_to_pygame_image(np_image):
    np_img = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
    return pygame.image.frombuffer(np_img.tobytes(), np_img.shape[1::-1], "RGB")


def get_config(logfile):
    """
    Deserialize original config from logfile. Return None if missing
    """
    log = LogReader(logfile, only_stream_id=0)
    print("original args:", next(log)[-1])  # old arguments
    config_str = next(log)[-1]
    return literal_eval(config_str.decode('ascii'))


class Frame:
    """Just single frame with collected data"""
    def __init__(self):
        self.lidar_up = None
        self.lidar_down = None


class Framer:
    """Creates frames from log entries. Packs together closest scan, pose and camera picture."""
    def __init__(self, filepath, lidar_name=None, lidar2_name=None, pose2d_name=None, pose3d_name=None, camera_name=None,
                 camera2_name=None, bbox_name=None, rgbd_name=None, joint_name=None, keyframes_name=None, title_name=None,
                 lidar_up_name=None, lidar_down_name=None):
        self.log = LogIndexedReader(filepath)
        self.current = 0
        self.frame = Frame()
        self.pose = [0, 0, 0]
        self.pose2d = [0, 0, 0]
        self.pose3d = [[0, 0, 0],[1, 0, 0, 0]]
        self.scan = []
        self.scan2 = []
        self.image = None
        self.image2 = None
        self.bbox = None
        self.joint = None
        self.keyframe = None
        self.title = None
        self.lidar_id, self.pose2d_id, self.pose3d_id, self.camera_id = None, None, None, None
        self.lidar2_id = None
        self.camera2_id = None
        self.bbox_id = None
        self.rgbd_id = None
        self.joint_id = None
        self.keyframes_id = None
        self.title_id = []
        self.lidar_up_id = None
        self.lidar_down_id = None
        names = lookup_stream_names(filepath)
        if lidar_name is not None:
            self.lidar_id = names.index(lidar_name) + 1
        if lidar2_name is not None:
            self.lidar2_id = names.index(lidar2_name) + 1
        if pose2d_name is not None:
            self.pose2d_id = names.index(pose2d_name) + 1
        if pose3d_name is not None:
            self.pose3d_id = names.index(pose3d_name) + 1
        if camera_name is not None:
            self.camera_id = names.index(camera_name) + 1
        if camera2_name is not None:
            self.camera2_id = names.index(camera2_name) + 1
        if bbox_name is not None:
            self.bbox_id = names.index(bbox_name) + 1
        if rgbd_name is not None:
            self.rgbd_id = names.index(rgbd_name) + 1
        if joint_name is not None:
            self.joint_id = names.index(joint_name) + 1
        if keyframes_name is not None:
            self.keyframes_id = names.index(keyframes_name) + 1
        if title_name is not None:
            title_list = title_name.split(",")
            for ttn in title_list:
                self.title_id.append(names.index(ttn) + 1)
        if lidar_up_name is not None:
            self.lidar_up_id = names.index(lidar_up_name) + 1
        if lidar_down_name is not None:
            self.lidar_down_id = names.index(lidar_down_name) + 1

    def __enter__(self):
        self.log.__enter__()
        return self

    def __exit__(self, *args):
        self.log.__exit__()

    def prev(self):
        return self._step(-1)

    def next(self):
        return self._step(1)

    def seek(self, desired_timestamp):
        print('Seek:', desired_timestamp)
        timestamp, stream_id, data = self.log[self.current]
        print('current', timestamp)
        if timestamp > desired_timestamp:
            return timestamp
        start = self.current
        end = len(self.log) - 1
        timestamp, stream_id, data = self.log[end]
        print('end', timestamp)
        while timestamp < desired_timestamp:
            self.log.grow()
            end = len(self.log) - 1
            timestamp, stream_id, data = self.log[end]
            print('end', timestamp)
        while start + 1 < end:
            mid = (start + end)//2
            timestamp, stream_id, data = self.log[mid]
            if timestamp < desired_timestamp:
                start = mid
            else:
                end = mid
        print('selected', timestamp)
        self.current = start



    def _step(self, direction):
        self.bbox = []
        self.title = []
        if (self.current + direction) >= len(self.log):
            self.log.grow()
        while self.current + direction >= 0 and self.current + direction < len(self.log):
            self.current += direction
            timestamp, stream_id, data = self.log[self.current]
            if stream_id == self.keyframes_id:
                self.keyframe = True
            if stream_id in self.title_id and stream_id != self.bbox_id:
                self.title.append(deserialize(data))
            if stream_id == self.lidar_id:
                self.scan = deserialize(data)
                keyframe = self.keyframe
                self.keyframe = False
                return timestamp, self.frame, self.pose, self.pose3d, self.scan, self.scan2, self.image, self.image2, self.bbox, self.joint, keyframe, self.title, False
            if stream_id == self.lidar2_id:
                self.scan2 = deserialize(data)
            if stream_id == self.lidar_up_id:
                self.frame.lidar_up = deserialize(data)
            if stream_id == self.lidar_down_id:
                self.frame.lidar_down = deserialize(data)
            elif stream_id == self.camera_id:
                self.image, _ = get_image(deserialize(data))
                # bounding boxes associated with an image are stored after the image in the log
                # therefore, we need to continue reading the log past the image in order to gathering its bounding box data
                current = self.current
                while current + direction >= 0 and current + direction < len(self.log):
                    current += direction
                    _, new_stream_id, new_data = self.log[current]
                    if new_stream_id == self.bbox_id:
                        self.bbox.append(deserialize(new_data))
                    if new_stream_id in self.title_id:
                        self.title.append(deserialize(new_data))
                    if new_stream_id == self.camera_id:
                        break
                if self.lidar_id is None:
                    keyframe = self.keyframe
                    self.keyframe = False
                    return timestamp, self.frame, self.pose, self.pose3d, self.scan, self.scan2, self.image, self.image2, self.bbox, self.joint, keyframe, self.title, False
            elif stream_id == self.camera2_id:
                self.image2, _ = get_image(deserialize(data))
            elif stream_id == self.rgbd_id:
                _, _, img_data, depth_data = deserialize(data)
                self.image, self.image2 = get_image((img_data, depth_data))
                if self.lidar_id is None:
                    keyframe = self.keyframe
                    self.keyframe = False
                    return timestamp, self.frame, self.pose, self.pose3d, self.scan, self.scan2, self.image, self.image2, self.bbox, self.joint, keyframe, self.title, False
            elif stream_id == self.joint_id:
                self.joint = deserialize(data)
            elif stream_id in [self.pose2d_id, self.pose3d_id]:
                if stream_id == self.pose3d_id:
                    pose3d, orientation = deserialize(data)
                    assert len(pose3d) == 3
                    assert len(orientation) == 4
                    self.pose = [pose3d[0], pose3d[1], quaternion.heading(orientation)]
                    self.pose3d = [pose3d, orientation]
                else:
                    arr = deserialize(data)
                    assert len(arr) == 3
                    self.pose = (arr[0] / 1000.0, arr[1] / 1000.0, math.radians(arr[2] / 100.0))
                x, y, heading = self.pose
                self.pose = (x * math.cos(g_rotation_offset_rad) - y * math.sin(g_rotation_offset_rad),
                             x * math.sin(g_rotation_offset_rad) + y * math.cos(g_rotation_offset_rad),
                             heading + g_rotation_offset_rad)
                if self.lidar_id is None and self.camera_id is None:
                    keyframe = self.keyframe
                    self.keyframe = False
                    return timestamp, self.frame, self.pose, self.pose3d, self.scan, self.scan2, self.image, self.image2, self.bbox, self.joint, keyframe, self.title, False
        return timedelta(), self.frame, self.pose, self.pose3d, self.scan, self.scan2, self.image, self.image2, self.bbox, self.joint, self.keyframe, self.title, True


def lidarview(gen, caption_filename, callback=False, callback_img=False, out_video=None, jump=None):
    global g_scale, WINDOW_SIZE
    last_timestamp = None
    last_image = None

    if out_video is not None:
        width, height = WINDOW_SIZE
        fps = 10
        writer = cv2.VideoWriter(out_video,
                                 cv2.VideoWriter_fourcc('F', 'M', 'P', '4'),
                                 fps,
                                 (width, height))

    pygame.display.init()
    pygame.font.init()
    screen = pygame.display.set_mode(WINDOW_SIZE, pygame.RESIZABLE)

    # create backgroud
    background = pygame.Surface(screen.get_size())
#    background.set_colorkey((0,0,0))

    # create foreground
    foreground = pygame.Surface(screen.get_size())
#    foreground.set_colorkey((0,0,0))

    # display everything
    screen.blit(background, (0, 0))
    screen.blit(foreground, (0, 0))
    pygame.display.flip()

    pygame.key.set_repeat(200, 60)

    paused = False
    camera_on = True
    use_image2 = False
    map_on = False
    poses = []
    acc_pts = []
    grid = defaultdict(int)
    skip_frames = 0
    frames_step = 0
    save_counter = 0
    was_resized = False

    history = gen
    max_timestamp = None
    wait_for_keyframe = False

    if jump is not None:
        gen.seek(timedelta(seconds=jump))

    while True:
        timestamp, frame, pose, pose3d, scan, scan2, image, image2, bbox, joint, keyframe, title, eof = history.next()

        if max_timestamp is None or max_timestamp < timestamp:
            # build map only for new data
            max_timestamp = timestamp

            # remove potential duplicity of poses (i.e. when not moving)
            if len(poses) == 0 or math.hypot(poses[-1][0] - pose[0], poses[-1][1] - pose[1]) >= TAIL_MIN_STEP:
                poses.append(pose)

            xy_scan = scan2xy(pose, scan)
            xy_scan_filtred = filter_pts(grid, xy_scan)
            acc_pts.extend(xy_scan_filtred)

        if wait_for_keyframe and not keyframe and not eof:
            paused = True
            caption = caption_filename + ": %s" % timestamp
            caption += ' searching fwd ...'
            pygame.display.set_caption(caption)
            event = pygame.event.poll()
            if event.type != KEYDOWN:
                continue
        wait_for_keyframe = False

        if skip_frames > 0 and not eof:
            skip_frames -= 1
            continue
        skip_frames = frames_step

        while True:
            caption = caption_filename + ": %s" % timestamp
            for t in title:
                caption += ' (' + str(t) + ')'
            if paused:
                caption += ' (PAUSED)'
            if frames_step > 0:
                caption += ' step=' + str(frames_step)
            if eof:
                caption += ' [END]'
            if keyframe:
                caption += ' [KEYFRAME]'
            pygame.display.set_caption(caption)

            foreground.fill((0, 0, 0))
            img = image2 if use_image2 else image
            if callback_img:
                if last_timestamp != timestamp:
                    img = numpy_to_pygame_image(callback_img(pygame_to_numpy_image(img)))
                    last_timestamp = timestamp
                    last_image = img
                else:
                    img = last_image
            draw_robot(foreground, pose, joint)
            draw_scan(foreground, pose, scan2, color=(128, 128, 0), joint=joint)
            draw_scan_up_down(foreground, pose, frame.lidar_up, frame.lidar_down, color=(255, 0, 0))
            draw(foreground, pose, scan, poses=poses,
                 image=img if camera_on else None, bbox=bbox,
                 acc_pts=acc_pts if map_on else None,
                 callback=callback)
            screen.blit(background, (0, 0))
            screen.blit(foreground, (0, 0))
            pygame.display.flip()

            if out_video is not None:
                # https://stackoverflow.com/questions/53101698/how-to-convert-a-pygame-image-to-open-cv-image/53108946#53108946
                #  create a copy of the surface
                view = pygame.surfarray.array3d(screen)
                #  convert from (width, height, channel) to (height, width, channel)
                view = view.transpose([1, 0, 2])
                #  convert from rgb to bgr
                img_bgr = cv2.cvtColor(view, cv2.COLOR_RGB2BGR)
                writer.write(img_bgr)

            if paused or eof:
                event = pygame.event.wait()
            else:
                event = pygame.event.poll()
            if event.type == QUIT:
                return
            if event.type == pygame.VIDEORESIZE:
                was_resized = True
                updated_size = event.size
            if event.type is pygame.ACTIVEEVENT and was_resized:
                was_resized = False
                screen = pygame.display.set_mode(updated_size, pygame.RESIZABLE)
                WINDOW_SIZE = updated_size
                background = pygame.Surface(screen.get_size())
                foreground = pygame.Surface(screen.get_size())
            if event.type == KEYDOWN:
                if event.key in [K_ESCAPE, K_q]:
                    return
                if event.key == K_SPACE:
                    paused = not paused
                if event.key in [K_PLUS, K_KP_PLUS, K_EQUALS]:
                    # K_EQUALS is for convenience: on notebook keyboard is '+' as Shift+'='
                    g_scale *= 2.0
                if event.key in [K_MINUS, K_KP_MINUS]:
                    g_scale /= 2.0
                if event.key == K_c:
                    if pygame.key.get_mods() & pygame.KMOD_LSHIFT:
                        use_image2 = not use_image2
                    else:
                        camera_on = not camera_on
                if event.key == K_m:
                    map_on = not map_on
                if event.key == K_0:
                    frames_step = 0
                if event.key == K_1:
                    frames_step = 10
                if event.key == K_2:
                    frames_step = 20
                if event.key == K_3:
                    frames_step = 30
                if event.key == K_4:
                    frames_step = 40
                if event.key == K_9:
                    frames_step = 90
                if event.key == K_s:
                    save_image = image2 if use_image2 else image
                    pygame.image.save(save_image, g_prefix + "-{:04}.jpg".format(save_counter))
                    save_counter += 1
                if event.key == K_b:  # swap binary danger image on/off
                    global g_danger_binary_image
                    g_danger_binary_image = not g_danger_binary_image
                    history.prev()
                if event.key == K_d:  # dump scan
                    print(scan)
                    np.savez_compressed('depth.npz', depth=g_depth, pose_xyz=pose3d[0], pose_quat = pose3d[1],
                                        img=pygame_to_numpy_image(image), scan=scan)
                if event.key == K_p:  # print position
                    print(pose)
                    x, y, heading = pose
                    if math.hypot(x, y) > 0.1:
                        print('rotation (deg) =', math.degrees(heading), 'dist =', math.hypot(x, y))
                    else:
                        print('rotation not available')
                if event.key == K_n:  # next keyframe
                    aborted = False
                    if pygame.key.get_mods() & pygame.KMOD_LSHIFT:  # previous keyframe
                        if keyframe:
                            history.prev()  # search for the previous one
                        data = history.prev()
                        while not data[-2] and not data[-1]:
                            # EOF is at the beginning as well as at the end
                            data = history.prev()
                            caption = caption_filename + ": %s" % data[0]
                            caption += ' searching bwd ...'
                            pygame.display.set_caption(caption)
                            tmp_event = pygame.event.poll()
                            if tmp_event.type == KEYDOWN:
                                aborted = True
                                timestamp = data[0]
                                break
                        history.prev()
                    if aborted:
                        paused = True
                    else:
                        wait_for_keyframe = True
                        paused = False  # let it search for next keyframe

                if event.key == K_RIGHT:
                    break
                if event.key == K_LEFT:
                    paused = True
                    history.prev()
                    history.prev()
                    break
            if event.type == pygame.NOEVENT and not paused and not eof:
                break
    if out_video is not None:
        writer.release()


def main(args_in=None, startswith=None):
    import argparse
    import os.path
    global g_rotation_offset_rad, g_lidar_fov_deg, MAX_SCAN_LIMIT, WINDOW_SIZE, g_prefix, g_log_config

    parser = argparse.ArgumentParser(description='View lidar scans')
    parser.add_argument('logfile', help='recorded log file')

    parser.add_argument('--lidar', help='stream ID')
    parser.add_argument('--lidar2', help='stream ID of second lidar (back or slope)')
    parser.add_argument('--lidar-limit', help='display scan limit in millimeters',
                        type=int, default=MAX_SCAN_LIMIT)
    parser.add_argument('--lidar-up', help='drone point lidar up stream ID')
    parser.add_argument('--lidar-down', help='drone point lidar down stream ID')
    pose = parser.add_mutually_exclusive_group()
    pose.add_argument('--pose2d', help='stream ID for pose2d messages')
    pose.add_argument('--pose3d', help='stream ID for pose3d messages')

    parser.add_argument('--camera', help='stream ID for JPEG images')
    parser.add_argument('--camera2', help='stream ID for 2nd JPEG images')
    parser.add_argument('--bbox', help='stream ID for detection bounding box')

    parser.add_argument('--rgbd', help='stream ID for RGBD')

    parser.add_argument('--joint', help='stream ID joint angle for articulated robots (Kloubak)')

    parser.add_argument('--keyframes', help='stream ID typically for artifacts detection')
    parser.add_argument('--title', help='stream ID of data to be displayed in title')

    parser.add_argument('--window-size', help='set window size in pixels', type=int, nargs=2)

    parser.add_argument('--callback', help='callback function for lidar scans')

    parser.add_argument('--callback-img', help='callback function for image, eg.: subt.tf_detector:TfDetector')

    parser.add_argument('--rotate', help='rotate poses by angle in degrees, offset',
                        type=float, default=0.0)

    parser.add_argument('--deg', help='lidar field of view in degrees',
                        type=float, default=270)

    parser.add_argument('--jump', '-j', help='jump to given time in seconds', type=int)

    parser.add_argument('--create-video', help='filename of output video')
    parser.add_argument('--im-prefix', help='prefix for saved images')

    args = parser.parse_args(args_in)

    p = pathlib.Path(args.logfile)
    if p.is_dir():
        if startswith is not None:
            g = iter(child for child in p.iterdir() if child.name.startswith(startswith))
        else:
            g = p.iterdir()
        args.logfile = max(g, key=lambda a: a.stat().st_mtime)
        print(args.logfile)

    if not any([args.lidar, args.pose2d, args.pose3d, args.camera, args.rgbd]):
        print("Available streams:")
        for stream in lookup_stream_names(args.logfile):
            print("  ", stream)
        return

    g_log_config = get_config(args.logfile)

    callback = None
    if args.callback is not None:
        callback = get_class_by_name(args.callback)
    callback_img = None
    if args.callback_img:
        name = args.callback_img
        assert ':' in name, name  # import path and class name expected
        s = name.split(':')
        assert len(s) == 2, s  # package and class name
        module_name, class_name = s
        m = import_module(module_name)
        callback_img = getattr(m, class_name)
        callback_img = callback_img().run_on_image

    if args.lidar_limit is not None:
        MAX_SCAN_LIMIT = args.lidar_limit
    if args.window_size is not None:
        WINDOW_SIZE = args.window_size
    if args.im_prefix:
        g_prefix = args.im_prefix

    filename = os.path.basename(args.logfile)
    g_rotation_offset_rad = math.radians(args.rotate)
    g_lidar_fov_deg = args.deg
    with Framer(args.logfile, lidar_name=args.lidar, lidar2_name=args.lidar2, pose2d_name=args.pose2d, pose3d_name=args.pose3d,
                camera_name=args.camera, camera2_name=args.camera2, bbox_name=args.bbox, rgbd_name=args.rgbd, joint_name=args.joint,
                keyframes_name=args.keyframes, title_name=args.title, lidar_up_name=args.lidar_up, lidar_down_name=args.lidar_down) as framer:
        lidarview(framer, caption_filename=filename, callback=callback, callback_img=callback_img, out_video=args.create_video, jump=args.jump)

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
