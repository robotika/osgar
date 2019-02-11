"""
  Draw laser scans
"""
import math
import io
from datetime import timedelta
from ast import literal_eval
from collections import defaultdict

import pygame
from pygame.locals import *

from osgar.logger import LogReader, lookup_stream_id, lookup_stream_names
from osgar.lib.serialize import deserialize
from osgar.lib.config import get_class_by_name


WINDOW_SIZE = 1200, 660
TAIL_MIN_STEP = 0.1  # in meters
g_scale = 30


def scans_gen(logfile, lidar_name=None, poses_name=None, camera_name=None):
    """
    Generator for (timestamp, pose, lidar, image) where freqency is defined
    by LIDAR and pose and image is used the most recent
    """
    names = lookup_stream_names(logfile)
    assert not(lidar_name is None and poses_name is None and camera_name is None), names

    lidar_id, poses_id, camera_id = None, None, None

    if lidar_name is not None:
        lidar_id = names.index(lidar_name) + 1
    if poses_name is not None:
        poses_id = names.index(poses_name) + 1
    if camera_name is not None:
        camera_id = names.index(camera_name) + 1

    pose = (0, 0, 0)
    image = None
    scan = []
    eof = False
    streams = [s for s in [lidar_id, poses_id, camera_id] if s is not None]
    with LogReader(logfile, only_stream_id=streams) as log:
        for timestamp, stream_id, data in log:
            if stream_id == lidar_id:
                scan = deserialize(data)
                yield timestamp, pose, scan, image, eof
            elif stream_id == camera_id:
                jpeg = deserialize(data)
                image = pygame.image.load(io.BytesIO(jpeg), 'JPG').convert()
                if lidar_id is None:
                    yield timestamp, pose, scan, image, eof
            elif stream_id == poses_id:
                arr = deserialize(data)
                assert len(arr) == 3
                pose = (arr[0]/1000.0, arr[1]/1000.0, math.radians(arr[2]/100.0))
                if lidar_id is None and camera_id is None:
                    yield timestamp, pose, scan, image, eof

    # generate last message with EOF ...
    eof = True
    yield timestamp, pose, scan, image, eof


def scans_gen_legacy(logfile):
    # old LIDAR text format
    timestamp = None
    start_time_sec = None
    pose = (0, 0, 0)
    image = None
    for line in open(logfile):
        if line.startswith('['):
            arr = literal_eval(line)
            yield timestamp, pose, arr, image, False
        else:
            s = line.split()
            if len(s) == 2:
                time_sec = float(s[1])
                if start_time_sec is None:
                    start_time_sec = time_sec
                timestamp = timedelta(seconds = time_sec - start_time_sec)

    yield timestamp, pose, arr, image, True  # EOF


def scr(x, y):
    global g_scale
    return WINDOW_SIZE[0]//2 + round(g_scale*x), WINDOW_SIZE[1]//2 - round(g_scale*y)


def scan2xy(pose, scan):
    X, Y, heading = pose
    pts = []
    for i, i_dist in enumerate(scan):
        if i_dist == 0 or i_dist >= 10000:
            continue
        angle = math.radians(270 * (i / len(scan)) - 135) + heading
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
        angle = math.radians(270 * (i / len(scan)) - 135) + heading
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
        cameraView = pygame.transform.scale(image, (512, 384))
        foreground.blit(cameraView, (0, 0))


    if callback is not None:
        debug_poly = []
        callback(scan, debug_poly)
        for poly in debug_poly:
            prev = None
            pts = [scr(x, y) for x, y in poly]
            for a, b in zip(pts[:-1], pts[1:]):
                pygame.draw.line(foreground, (200, 200, 0), a, b, 1)


def lidarview(gen, callback=False):
    global g_scale

    pygame.init()    
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
    poses = []
    acc_pts = []
    skip_frames = 0
    frames_step = 0
    for timestamp, pose, scan, image, eof in gen:
        # remove potential duplicity of poses (i.e. when not moving)
        if len(poses) == 0 or math.hypot(poses[-1][0] - pose[0], poses[-1][1] - pose[1]) >= TAIL_MIN_STEP:
            poses.append(pose)

        acc_pts.extend(scan2xy(pose, scan))
        acc_pts = filter_pts(acc_pts)

        if skip_frames > 0 and not eof:
            skip_frames -= 1
            continue
        skip_frames = frames_step

        while True:
            caption = "Time %s" % timestamp
            if paused:
                caption += ' (PAUSED)'
            if frames_step > 0:
                caption += ' step=' + str(frames_step)
            if eof:
                caption += ' [END]'
            pygame.display.set_caption(caption)

            foreground.fill((0, 0, 0))
            if camera_on:
                draw(foreground, pose, scan, poses=poses, image=image, callback=callback)
            else:
                draw(foreground, pose, scan, poses=poses, callback=callback, acc_pts=acc_pts)
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
                    camera_on = not camera_on
                if event.key == K_0:
                    frames_step = 0
                    sleep_time = 100
                if event.key == K_1:
                    frames_step = 10
                    sleep_time = 10
                if event.key == K_2:
                    frames_step = 20
                    sleep_time = 10

            if event.type == pygame.NOEVENT and not paused and not eof:
                break


def main():
    import argparse

    parser = argparse.ArgumentParser(description='View lidar scans')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--legacy', help='use old text lidar log format', action='store_true')

    parser.add_argument('--lidar', help='stream ID')
    parser.add_argument('--poses', help='stream ID for pose2d messages')
    parser.add_argument('--camera', help='stream ID for JPEG images')

    parser.add_argument('--callback', help='callback function for lidar scans')

    args = parser.parse_args()

    callback = None
    if args.callback is not None:
        callback = get_class_by_name(args.callback)

    if args.legacy:
        lidarview(scans_gen_legacy(args.logfile), callback=callback)
    else:
        lidarview(scans_gen(args.logfile, lidar_name=args.lidar,
                            poses_name=args.poses, camera_name=args.camera),
                  callback=callback)

if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 

