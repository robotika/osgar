"""
  Draw laser scans
"""
import math
from datetime import timedelta
from ast import literal_eval

import pygame
from pygame.locals import *

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


WINDOW_SIZE = 1200, 660


def scans_gen(logfile, stream_id):
    only_stream = lookup_stream_id(logfile, stream_id)
    with LogReader(logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            scan = deserialize(data)
            yield timestamp, scan


def scans_gen_legacy(logfile):
    # old LIDAR text format
    timestamp = None
    start_time_sec = None
    for line in open(logfile):
        if line.startswith('['):
            arr = literal_eval(line)
            yield timestamp, arr
        else:
            s = line.split()
            if len(s) == 2:
                time_sec = float(s[1])
                if start_time_sec is None:
                    start_time_sec = time_sec
                timestamp = timedelta(seconds = time_sec - start_time_sec)


def scr(x, y):
    scale = 30
    return WINDOW_SIZE[0]//2 + round(scale*x), WINDOW_SIZE[1]//2 - round(scale*y)


def draw(foreground, scan):
    color = (0, 255, 0)
    for i, i_dist in enumerate(scan):
        if i_dist == 0 or i_dist >= 10000:
            continue
        angle = math.radians(i * 270 / len(scan))
        dist = i_dist/1000.0
        x, y = dist * math.cos(angle), dist * math.sin(angle)
        pygame.draw.circle(foreground, color, scr(x, y), 3)


def lidarview(gen):
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
    for timestamp, scan in gen:
        if max(scan) == 0:
            continue
        foreground.fill((0, 0, 0))
        draw(foreground, scan)
        screen.blit(background, (0, 0))
        screen.blit(foreground, (0, 0))
        pygame.display.flip() 

        while True:
            caption = "Time %s" % timestamp
            if paused:
                caption += ' (PAUSED)'
            pygame.display.set_caption(caption)

            event = pygame.event.wait()
            if event.type == QUIT:
                return
            if event.type == KEYDOWN:
                if event.key in [K_ESCAPE, K_q]:
                    return
                if event.key == K_SPACE:
                    paused = not paused
            if event.type == timer_event and not paused:
                break


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='View lidar scans')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--stream', help='stream ID', default='lidar.scan')
    parser.add_argument('--legacy', help='use old text lidar log format', action='store_true')
    args = parser.parse_args()

    if args.legacy:
        lidarview(scans_gen_legacy(args.logfile))
    else:
        lidarview(scans_gen(args.logfile, args.stream))

# vim: expandtab sw=4 ts=4 

