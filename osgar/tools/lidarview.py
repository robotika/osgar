"""
  Draw laser scans
"""
import math
import io
from datetime import timedelta
from ast import literal_eval
from collections import defaultdict

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "1"
import pygame
from pygame.locals import *

from osgar.logger import LogReader, LogIndexedReader, lookup_stream_id, lookup_stream_names
from osgar.lib.serialize import deserialize
from osgar.lib.config import get_class_by_name
from osgar.lib import quaternion


#WINDOW_SIZE = 1200, 660
WINDOW_SIZE = 1600, 1000
TAIL_MIN_STEP = 0.1  # in meters
HISTORY_SIZE = 100

g_scale = 30
g_rotation_offset_rad = 0.0  # set by --rotation (deg)
g_lidar_fov_deg = 270  # controlled by --deg (deg)


def scr(x, y):
    global g_scale
    return WINDOW_SIZE[0]//2 + round(g_scale*x), WINDOW_SIZE[1]//2 - round(g_scale*y)


def scan2xy(pose, scan):
    X, Y, heading = pose
    pts = []
    for i, i_dist in enumerate(scan):
        if i_dist == 0 or i_dist >= 10000:
            continue
        angle = math.radians(g_lidar_fov_deg * (i / len(scan)) - g_lidar_fov_deg/2) + heading
        dist = i_dist/1000.0
        x, y = dist * math.cos(angle), dist * math.sin(angle)
        pts.append((X + x, Y + y))
    return pts


def filter_pts(pts):
    # filter out duplicity points - keep older
    grid = defaultdict(int)
    ret = []
    for x, y in pts:
        k = int(2*x), int(2*y)
        grid[k] += 1
        if grid[k] < 10:
            ret.append((x, y))
    return ret


def draw(foreground, pose, scan, poses=[], image=None, callback=None, acc_pts=None):
    color = (0, 255, 0)
    X, Y, heading = pose
    for i, i_dist in enumerate(scan):
        if i_dist == 0 or i_dist >= 10000:
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
    pygame.draw.circle(foreground, (255, 128, 0), scr(0, 0), 5)
    pygame.draw.line(foreground, (255, 128, 0), scr(0, 0), scr(0.5*math.cos(heading), 0.5*math.sin(heading)), 1)

    # draw trace of poses
    for x, y, __ in poses:
        pygame.draw.circle(foreground, (255, 128, 0), scr(x - X, y - Y), 2)

    # draw accumulated points
    if acc_pts is not None:
        for x, y in acc_pts:
            pygame.draw.circle(foreground, (0, 127, 0), scr(x - X, y - Y), 2)

    if image is not None:
        width, height = image.get_size()
        if width > WINDOW_SIZE[0] or height > WINDOW_SIZE[1]:
            width, height = (512, 384)
        cameraView = pygame.transform.scale(image, (width, height))
        foreground.blit(cameraView, (0, 0))


    if callback is not None:
        debug_poly = []
        callback(scan, debug_poly)
        for poly in debug_poly:
            prev = None
            pts = [scr(x, y) for x, y in poly]
            for a, b in zip(pts[:-1], pts[1:]):
                pygame.draw.line(foreground, (200, 200, 0), a, b, 1)


class Framer:
    """Creates frames from log entries. Packs together closest scan, pose and camera picture."""
    def __init__(self, filepath, lidar_name=None, pose2d_name=None, pose3d_name=None, camera_name=None,
                 camera2_name=None, keyframes_name=None):
        self.log = LogIndexedReader(filepath)
        self.current = 0
        self.pose = [0, 0, 0]
        self.pose2d = [0, 0, 0]
        self.pose3d = [[0, 0, 0],[1, 0, 0, 0]]
        self.scan = []
        self.image = None
        self.image2 = None
        self.keyframe = None
        self.lidar_id, self.pose2d_id, self.pose3d_id, self.camera_id = None, None, None, None
        self.camera2_id = None
        self.keyframes_id = None
        names = lookup_stream_names(filepath)
        if lidar_name is not None:
            self.lidar_id = names.index(lidar_name) + 1
        if pose2d_name is not None:
            self.pose2d_id = names.index(pose2d_name) + 1
        if pose3d_name is not None:
            self.pose3d_id = names.index(pose3d_name) + 1
        if camera_name is not None:
            self.camera_id = names.index(camera_name) + 1
        if camera2_name is not None:
            self.camera2_id = names.index(camera2_name) + 1
        if keyframes_name is not None:
            self.keyframes_id = names.index(keyframes_name) + 1

    def __enter__(self):
        self.log.__enter__()
        return self

    def __exit__(self, *args):
        self.log.__exit__()

    def prev(self):
        return self._step(-1)

    def next(self):
        return self._step(1)

    def _step(self, direction):
        if (self.current + direction) >= len(self.log):
            self.log.grow()
        while self.current + direction >= 0 and self.current + direction < len(self.log):
            self.current += direction
            timestamp, stream_id, data = self.log[self.current]
            if stream_id == self.keyframes_id:
                self.keyframe = True
            if stream_id == self.lidar_id:
                self.scan = deserialize(data)
                keyframe = self.keyframe
                self.keyframe = False
                return timestamp, self.pose, self.scan, self.image, self.image2, keyframe, False
            elif stream_id == self.camera_id:
                jpeg = deserialize(data)
                self.image = pygame.image.load(io.BytesIO(jpeg), 'JPG').convert()
                if self.lidar_id is None:
                    keyframe = self.keyframe
                    self.keyframe = False
                    return timestamp, self.pose, self.scan, self.image, self.image2, keyframe, False
            elif stream_id == self.camera2_id:
                jpeg = deserialize(data)
                self.image2 = pygame.image.load(io.BytesIO(jpeg), 'JPG').convert()
            elif stream_id == self.pose3d_id:
                pose3d, orientation = deserialize(data)
                assert len(pose3d) == 3
                assert len(orientation) == 4
                self.pose = [pose3d[0], pose3d[1], quaternion.heading(orientation)]
                self.pose3d = [pose3d, orientation]
            elif stream_id == self.pose2d_id:
                arr = deserialize(data)
                assert len(arr) == 3
                self.pose = (arr[0]/1000.0, arr[1]/1000.0, math.radians(arr[2]/100.0))
                x, y, heading = self.pose
                self.pose = (x * math.cos(g_rotation_offset_rad) - y * math.sin(g_rotation_offset_rad),
                             x * math.sin(g_rotation_offset_rad) + y * math.cos(g_rotation_offset_rad),
                             heading + g_rotation_offset_rad)
                if self.lidar_id is None and self.camera_id is None:
                    keyframe = self.keyframe
                    self.keyframe = False
                    return timestamp, self.pose, self.scan, self.image, self.image2, keyframe, False
        return timedelta(), self.pose, self.scan, self.image, self.image2, self.keyframe, True



def lidarview(gen, caption_filename, callback=False):
    global g_scale

    pygame.display.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)

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

    pygame.key.set_repeat(200, 20)
    sleep_time = 100

    paused = False
    camera_on = True
    use_image2 = False
    map_on = False
    poses = []
    acc_pts = []
    skip_frames = 0
    frames_step = 0
    save_counter = 0

    history = gen
    max_timestamp = None
    wait_for_keyframe = False
    while True:
        timestamp, pose, scan, image, image2, keyframe, eof = history.next()

        if max_timestamp is None or max_timestamp < timestamp:
            # build map only for new data
            max_timestamp = timestamp

            # remove potential duplicity of poses (i.e. when not moving)
            if len(poses) == 0 or math.hypot(poses[-1][0] - pose[0], poses[-1][1] - pose[1]) >= TAIL_MIN_STEP:
                poses.append(pose)

            acc_pts.extend(scan2xy(pose, scan))
            acc_pts = filter_pts(acc_pts)

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
            draw(foreground, pose, scan, poses=poses,
                 image=img if camera_on else None,
                 acc_pts=acc_pts if map_on else None,
                 callback=callback)
            screen.blit(background, (0, 0))
            screen.blit(foreground, (0, 0))
            pygame.display.flip() 

            if paused or eof:
                event = pygame.event.wait()
            else:
                event = pygame.event.poll()
            if event.type == pygame.NOEVENT:
                pygame.time.wait(sleep_time)
            if event.type == QUIT:
                return
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
                    sleep_time = 100
                if event.key == K_1:
                    frames_step = 10
                    sleep_time = 10
                if event.key == K_2:
                    frames_step = 20
                    sleep_time = 10
                if event.key == K_3:
                    frames_step = 30
                    sleep_time = 10
                if event.key == K_4:
                    frames_step = 40
                    sleep_time = 10
                if event.key == K_9:
                    frames_step = 90
                    sleep_time = 10
                if event.key == K_s:
                    pygame.image.save(image, "saveX-{:04}.jpg".format(save_counter))
                    save_counter += 1
                if event.key == K_d:  # dump scan
                    print(scan)
                if event.key == K_p:  # print position
                    print(pose)
                    x, y, heading = pose
                    if math.hypot(x, y) > 0.1:
                        print('rotation (deg) =', math.degrees(math.atan2(y, x)), 'dist =', math.hypot(x, y))
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


def main():
    import argparse
    import os.path
    global g_rotation_offset_rad, g_lidar_fov_deg

    parser = argparse.ArgumentParser(description='View lidar scans')
    parser.add_argument('logfile', help='recorded log file')

    parser.add_argument('--lidar', help='stream ID')

    pose = parser.add_mutually_exclusive_group()
    pose.add_argument('--pose2d', help='stream ID for pose2d messages')
    pose.add_argument('--pose3d', help='stream ID for pose3d messages')

    parser.add_argument('--camera', help='stream ID for JPEG images')
    parser.add_argument('--camera2', help='stream ID for 2nd JPEG images')

    parser.add_argument('--keyframes', help='stream ID typically for artifacts detection')

    parser.add_argument('--callback', help='callback function for lidar scans')

    parser.add_argument('--rotate', help='rotate poses by angle in degrees, offset',
                        type=float, default=0.0)

    parser.add_argument('--deg', help='lidar field of view in degrees',
                        type=float, default=270)

    args = parser.parse_args()
    if not any([args.lidar, args.pose2d, args.pose3d, args.camera]):
        print("Available streams:")
        for stream in lookup_stream_names(args.logfile):
            print("  ", stream)
        return

    callback = None
    if args.callback is not None:
        callback = get_class_by_name(args.callback)

    filename = os.path.basename(args.logfile)
    g_rotation_offset_rad = math.radians(args.rotate)
    g_lidar_fov_deg = args.deg
    with Framer(args.logfile, lidar_name=args.lidar, pose2d_name=args.pose2d, pose3d_name=args.pose3d,
                camera_name=args.camera, camera2_name=args.camera2, keyframes_name=args.keyframes) as framer:
        lidarview(framer, caption_filename=filename, callback=callback)

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 
