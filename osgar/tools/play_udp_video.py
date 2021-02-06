"""
  Play UDP video packets from Tello Drone
"""
from time import sleep
import socket
from threading import Thread

import cv2

from osgar.logger import LogReader, lookup_stream_names
from osgar.lib.serialize import deserialize

HOST = '127.0.0.1'
PORT = 11111


def read_thread():
    cap = cv2.VideoCapture(f'udp://@{HOST}:{PORT}')
    i = 0
    video = None
    while True:
        grabbed, img = cap.read()
        print(i)
        i += 1
        cv2.imshow('Paula', img)
        key = cv2.waitKey(100)
        if not grabbed or key == 27:
            break
        if video is None:
            height, width, _ = img.shape
            video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))
        video.write(img)

    if video is not None:
        video.release()
    cap.release()


def create_video(filename, output_name):
    reader = Thread(target=read_thread)
    reader.start()
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    names = lookup_stream_names(filename)
    print(names)
    ids = [i + 1 for i, name in enumerate(names) if 'video' in name]
    print(ids)
    prev_time = None
    for timestamp, channel_index, data_raw in LogReader(filename,
            only_stream_id=ids):
        channel = names[channel_index - 1]
        data = deserialize(data_raw)
        # reuse timestamp
        if prev_time is not None:
            sleep((timestamp - prev_time).total_seconds())
        prev_time = timestamp
        soc.sendto(data, (HOST, PORT))
    print('Replay completed!')
    reader.join()
    print('Reading completed!')


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='Logfile with H264 frames')
    parser.add_argument('--out', help='Video output filename', default='out.avi')
    args = parser.parse_args()

    create_video(args.logfile, args.out)

# vim: expandtab sw=4 ts=4
