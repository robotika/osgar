"""
  Draw laser scans
"""
import math
import io
from datetime import timedelta
from ast import literal_eval

import pygame
from pygame.locals import *

from osgar.logger import LogReader, lookup_stream_id, lookup_stream_names
from osgar.lib.serialize import deserialize


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
    streams = [s for s in [lidar_id, poses_id, camera_id] if s is not None]
    with LogReader(logfile, only_stream_id=streams) as log:
        for timestamp, stream_id, data in log:
            if stream_id == lidar_id:
                scan = deserialize(data)
                yield timestamp, pose, scan, image
            elif stream_id == camera_id:
                jpeg = deserialize(data)
                image = pygame.image.load(io.BytesIO(jpeg), 'JPG').convert()
                if lidar_id is None:
                    yield timestamp, pose, scan, image
            elif stream_id == poses_id:
                arr = deserialize(data)
                assert len(arr) == 3
                pose = (arr[0]/1000.0, arr[1]/1000.0, math.radians(arr[2]/100.0))
                if lidar_id is None and camera_id is None:
                    yield timestamp, pose, scan, image


def scans_gen_legacy(logfile):
    # old LIDAR text format
    timestamp = None
    start_time_sec = None
    pose = (0, 0, 0)
    image = None
    for line in open(logfile):
        if line.startswith('['):
            arr = literal_eval(line)
            yield timestamp, pose, arr, image
        else:
            s = line.split()
            if len(s) == 2:
                time_sec = float(s[1])
                if start_time_sec is None:
                    start_time_sec = time_sec
                timestamp = timedelta(seconds = time_sec - start_time_sec)


def scr(x, y):
    global g_scale
    return WINDOW_SIZE[0]//2 + round(g_scale*x), WINDOW_SIZE[1]//2 - round(g_scale*y)


def draw(foreground, pose, scan, poses=[], image=None):
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

    if image is not None:
        cameraView = pygame.transform.scale(image, (512, 384))
        foreground.blit( cameraView, (0, 0))

def lidarview(gen):
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
    timer_event = pygame.USEREVENT + 1
    pygame.time.set_timer(timer_event, 100)

    paused = False
    camera_on = True
    poses = []
    for timestamp, pose, scan, image in gen:
        # remove potential duplicity of poses (i.e. when not moving)
        if len(poses) == 0 or math.hypot(poses[-1][0] - pose[0], poses[-1][1] - pose[1]) >= TAIL_MIN_STEP:
            poses.append(pose)

        while True:
            caption = "Time %s" % timestamp
            if paused:
                caption += ' (PAUSED)'
            pygame.display.set_caption(caption)

            foreground.fill((0, 0, 0))
            if camera_on:
                draw(foreground, pose, scan, poses=poses, image=image)
            else:
                draw(foreground, pose, scan, poses=poses)
            screen.blit(background, (0, 0))
            screen.blit(foreground, (0, 0))
            pygame.display.flip() 

            event = pygame.event.wait()
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
            if event.type == timer_event and not paused:
                break


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='View lidar scans')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--legacy', help='use old text lidar log format', action='store_true')

    parser.add_argument('--lidar', help='stream ID')
    parser.add_argument('--poses', help='stream ID for pose2d messages')
    parser.add_argument('--camera', help='stream ID for JPEG images')
    args = parser.parse_args()

    if args.legacy:
        lidarview(scans_gen_legacy(args.logfile))
    else:
        lidarview(scans_gen(args.logfile, lidar_name=args.lidar,
                            poses_name=args.poses, camera_name=args.camera))

# vim: expandtab sw=4 ts=4 

