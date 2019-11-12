"""
  Log video stream provided by OpenCV camera
"""

import cv2
import trio


async def opencv(nursery, config, bus):
    port = config.get('port', 0)
    cap = await trio.to_thread.run_sync(cv2.VideoCapture, port)
    sleep = config.get('sleep')
    while bus.is_alive():
        ret, frame = await trio.to_thread.run_sync(cap.read)
        if ret:
            retval, data = await trio.to_thread.run_sync(cv2.imencode, '*.jpeg', frame)
            if len(data) > 0:
                await bus.publish('jpg', data.tobytes())
            if sleep is not None:
                await bus.sleep(sleep)
    cap.release()


if __name__ == "__main__":
    from osgar import record
    config = {
        "version": 2,
        "robot": {
            "modules": {
                "camera": {
                    "driver": "application",
                    "in": [],
                    "out": ["jpg"],
                    "init": {
                        "port": 0,
                        "sleep": 1.0
                    }
                }
            },
        "links": []
        }
    }
    log_prefix = "test-opencv-camera-"
    trio.run(record.record, config, log_prefix, 3, opencv)

# vim: expandtab sw=4 ts=4
