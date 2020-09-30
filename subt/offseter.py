from threading import Thread

class Offseter:

    def __init__(self, config, bus):
        bus.register("pose3d")
        self.thread = Thread(target=self.run)
        self.thread.name = bus.name
        self.bus = bus

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)

    def run(self):
        origin = None
        xyz = None
        while None in (origin, xyz):
            dt, channel, data = self.bus.listen()
            if channel == "origin":
                if len(data) == 8:
                    robot_name, x, y, z, qa, qb, qc, qd = data
                    origin = [x, y, z]
            elif channel == "pose3d":
                xyz, orientation = data
            else:
                raise RuntimeError(f"unknown channel '{channel}'")

        offset = [o - p for o,p in zip(origin, xyz)]

        while True:
            dt, channel, data = self.bus.listen()
            if channel == "origin" and len(data) == 8:
                robot_name, x, y, z, qa, qb, qc, qd = data
                origin = [x, y, z]
                offset = [o - p for o,p in zip(origin, xyz)]
            elif channel == "pose3d":
                xyz, orientation = data
                xyz_offset = [o + p for o,p in zip(offset, xyz)]
                self.bus.publish("pose3d", [xyz_offset, orientation])
            else:
                raise RuntimeError(f"unknown channel '{channel}'")


    def request_stop(self):
        self.bus.shutdown()


def main():
    import argparse
    import datetime
    from unittest.mock import MagicMock
    from osgar.logger import LogReader, lookup_stream_id
    from osgar.lib.serialize import deserialize
    from osgar.bus import Bus

    parser = argparse.ArgumentParser()
    parser.add_argument('logfile', help='filepath')
    args = parser.parse_args()

    origin_stream_id = lookup_stream_id(args.logfile, "rosmsg.origin")
    pose3d_stream_id = lookup_stream_id(args.logfile, "rospy.pose3d")
    channel = { origin_stream_id:"origin", pose3d_stream_id: "pose3d" }
    print(channel)

    bus = Bus(MagicMock(write=MagicMock(return_value=datetime.timedelta())))
    tester = bus.handle("tester")
    tester.register("pose3d", "origin")
    def aa():
        while True:
            dt, channel, data = tester.listen()
            print(channel, data)
    tester_thread = Thread(target=aa, name='tester')

    offseter_handle = bus.handle("offseter")
    offseter_node = Offseter({}, offseter_handle)

    bus.connect("tester.pose3d", "offseter.pose3d")
    bus.connect("tester.origin", "offseter.origin")
    bus.connect("offseter.pose3d", "testerer.pose3d")

    offseter_node.start()
    tester_thread.start()

    try:
        with LogReader(args.logfile, only_stream_id=[origin_stream_id, pose3d_stream_id]) as logreader:
            for time, stream, data in logreader:
                data = deserialize(data)
                tester.publish(channel[stream], data)
    except Exception as e:
        print(e)
        offseter_node.request_stop()
        tester.request_shutdown()
        offseter_node.join()
        tester.join()


if __name__ == "__main__":
    main()
