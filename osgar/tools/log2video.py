#!/usr/bin/python
"""
  Convert logfile to AVI video
"""
import pathlib
from datetime import timedelta
from subprocess import check_call

try:
    import cv2
except ImportError:
    print('\nERROR: Please install OpenCV\n    pip install opencv-python\n')

try:
    import numpy as np
except ImportError:
    print('\nERROR: Please install numpy\n    pip install numpy\n')

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize
from osgar.tools.log2wav import extract_audio_to_wav


def ffmpeg_video_processing(logfile, stream, audio, outfile):
    """
    Simple wrapper for H265 conversion with audio
    """
    # extract H.265 stream into separate file
    video_stream_id = lookup_stream_id(logfile, stream)
    audio_stream_id = lookup_stream_id(logfile, audio)
    video_1st_frame = None
    audio_1st_frame = None
    with open('tmp.h265', 'wb') as f:
        with LogReader(logfile, only_stream_id=video_stream_id) as log:
            for timestamp, stream_id, data in log:
                buf = deserialize(data)
                f.write(buf)
                if video_1st_frame is None:
                    video_1st_frame = timestamp

    with LogReader(logfile, only_stream_id=audio_stream_id) as log:
        for timestamp, stream_id, data in log:
            audio_1st_frame = timestamp
            break

    print(f'Video - audio difference: {video_1st_frame - audio_1st_frame}')
    extract_audio_to_wav(logfile, 'tmp.wav', audio,
                         rate=44100, channels=1, audio_format='paInt16')  # TODO extract from logfile

    diff_time_sec = (audio_1st_frame - video_1st_frame).total_seconds()
    cmd = f'ffmpeg -i tmp.h265 -itsoffset {diff_time_sec} -i tmp.wav -c:v copy -c:a aac -map 0:v:0 -map 1:a:0 {outfile}'
    cmd += ' -y -hide_banner -nostats'  # overwrite and less verbose
    print('Cmd:', cmd)
    check_call(cmd.split())


def create_video(logfile, stream, outfile, add_time=False,
                 start_time_sec=0, end_time_sec=None, fps=None,
                 flip=False, camera2=None, audio=None):
    if outfile is None:
        outfile = str(pathlib.Path(logfile).with_suffix(".mp4"))
    assert outfile.endswith(".mp4"), outfile
    only_stream = lookup_stream_id(logfile, stream)
    if camera2 is not None:
        dual_cam_id = lookup_stream_id(logfile, camera2)
        only_stream = [only_stream, dual_cam_id]
        img1, img2 = None, None
    else:
        dual_cam_id = None
    if audio is not None:
        assert start_time_sec == 0 and end_time_sec is None, (start_time_sec, end_time_sec)  # clipping is not supported
        assert fps is None  # FPS is currently not supported with audio
        return ffmpeg_video_processing(logfile, stream, audio, outfile)
    with LogReader(logfile, only_stream_id=only_stream) as log:
        writer = None
        for timestamp, stream_id, data in log:
            buf = deserialize(data)
            if isinstance(buf, np.ndarray):
                img_tmp = np.array(np.minimum(255 * 40, buf) / 40, dtype=np.uint8)
                img = cv2.applyColorMap(img_tmp, cv2.COLORMAP_JET)
            else:
                img = cv2.imdecode(np.fromstring(buf, dtype=np.uint8), 1)
            if flip:
                img = cv2.flip(img, 0)
            if dual_cam_id is not None:
                if stream_id == dual_cam_id:
                    img2 = img
                else:
                    img1 = img
                # make sure that both images are defined before you create writer
                if img1 is None or img2 is None:
                    continue
                img = np.concatenate((img1, img2), axis=1)
            if writer is None:
                height, width = img.shape[:2]
                writer = cv2.VideoWriter(outfile,
                                         cv2.VideoWriter_fourcc(*"mp4v"),
                                         fps,
                                         (width, height))
            if add_time:
                if (width, height) == (640, 480):
                    x, y = 350, 50
                    thickness = 3
                    size = 3.0
                else:
                    x, y = 800, 100
                    thickness = 5
                    size = 5.0
                # clip microseconds to miliseconds
                s = str(timestamp-timedelta(seconds=start_time_sec))[:-3]
                cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 
                            size, (255, 255, 255), thickness=thickness)

            if timestamp.total_seconds() > start_time_sec:
                writer.write(img)
            if end_time_sec is not None and timestamp.total_seconds() > end_time_sec:
                break
        writer.release()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Convert logfile to AVI video')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--stream', help='stream ID or name', default='camera.raw')
    parser.add_argument('--camera2', help='optional 2nd camera stream ID or name')
    parser.add_argument('--audio', help='stream ID or name for audio channel (if available)')
    parser.add_argument('--out', '-o', help='output AVI file', default=None)
    parser.add_argument('--display-time', '-t', help='add timestamp info', action='store_true')
    parser.add_argument('--start-time-sec', '-s', help='start video at later time (sec)',
                        type=float, default=0.0)
    parser.add_argument('--end-time-sec', '-e', help='cut video at given time (sec)',
                        type=float, default=None)
    parser.add_argument('--fps', help='frames per second', type=int)
    parser.add_argument('--flip', help='horizontal flip', action='store_true')
    args = parser.parse_args()

    create_video(args.logfile, args.stream, args.out, add_time=args.display_time,
                 start_time_sec=args.start_time_sec, end_time_sec=args.end_time_sec, fps=args.fps,
                 flip=args.flip, camera2=args.camera2, audio=args.audio)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 

