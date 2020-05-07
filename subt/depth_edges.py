import argparse

import numpy as np
import cv2
import osgar.logger
import osgar.lib.serialize

L = np.asarray([[0,1,0], [1, -4, 1], [0, 1, 0]], dtype=float)
print(L)

#import osgar.lib.depth
#print(osgar.lib.depth.pxs[0][:3], "...", osgar.lib.depth.pxs[0][-3:] )
#print(osgar.lib.depth.pys[:,0][:3], "...", osgar.lib.depth.pys[:,0][-3:])

class stepper:

    def __init__(self, logfile, streams):
        self.log = osgar.logger.LogIndexedReader(logfile)
        self.streams = streams
        self.imsize = (0, 1)
        self.current = 0

    def step(self, direction):
        while 0 < self.current + direction < len(self.log):
            self.current += direction
            timestamp, stream_id, data = self.log[self.current]
            if stream_id in self.streams:
                self.last = timestamp, stream_id, data
                break
        return self.last

    def __enter__(self):
        self.log.__enter__()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.log.__exit__(exc_type, exc_val, exc_tb)


class video_writer:

    def __init__(self, width, height):
        print(width, height)
        self.writer = cv2.VideoWriter("depth-edges.mkv", cv2.VideoWriter_fourcc(*"mp4v"), 50, (width, height))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.writer.release()

    def show(self, canvas):
        cv2.imshow("aaaa", canvas)
        key = cv2.waitKey(1)
        if key == ord('q'):
            raise SystemExit()
        self.writer.write(canvas)
        return 1


class img_shower:

    def __init__(self, width, height):
        self.timeout = 0

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

    def show(self, canvas):
        cv2.imshow("depth edges", canvas)
        key = cv2.waitKey(self.timeout)
        if key == 27 or key == ord('q'):
            raise SystemExit()
        if key == 83: # right
            return 1
        if key == 81: # left
            return -1
        if key == ord(' '):
            self.timeout = 0 if self.timeout > 0 else 1
        return 1


def main():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('-v', help='render video', action="store_true")
    args = parser.parse_args()

    depth_stream = osgar.logger.lookup_stream_id(args.logfile, 'rosmsg.depth')
    camera_stream = osgar.logger.lookup_stream_id(args.logfile, 'rosmsg.image')

    img_camera = None

    with stepper(args.logfile, streams=(depth_stream, camera_stream)) as log:
        while True:
            timestamp, stream_id, data = log.step(1)
            if stream_id == depth_stream:
                height, width = osgar.lib.serialize.deserialize(data).shape[:2]
                break
        log.current = 0
        direction = 1
        with (video_writer if args.v else img_shower)(width*2, height*2) as outputer:
            while True:
                timestamp, stream_id, data = log.step(direction)
                if stream_id == camera_stream:
                    jpeg = osgar.lib.serialize.deserialize(data)
                    img_camera = cv2.imdecode(np.fromstring(jpeg, dtype='uint8'), cv2.IMREAD_COLOR)
                else:
                    if img_camera is None:
                        continue
                    depth = osgar.lib.serialize.deserialize(data)
                    map = depth > 40*255
                    depth[map] = 40*255
                    edges = cv2.filter2D(depth, -1, L)
                    _, edges = cv2.threshold(edges, 1200, 20*255, cv2.THRESH_BINARY)
                    img_depth = np.asarray(depth/40, dtype='uint8')
                    img_edges = np.asarray(edges/40, dtype='uint8')
                    s1 = np.concatenate((img_depth, img_edges), axis=0)
                    s1 = cv2.cvtColor(s1, cv2.COLOR_GRAY2BGR)

                    img_spare = np.zeros(img_camera.shape, img_camera.dtype)
                    s2 = np.concatenate((img_camera, img_spare), axis=0)
                    canvas = np.concatenate((s1, s2), axis=1)
                    direction = outputer.show(canvas)


if __name__ == "__main__":
    main()
