"""
   Mapping server for Teambase to report maps to DARPA
   for SubT finals.
"""
import requests
import time
import cbor
import gzip
import struct

import numpy as np

from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_stream_id


URL_BASE = "http://localhost:8000"  # local docker test


class CommandPostRelay(object):
    def __init__(self, token):
        self.token = token
        self.map_url = URL_BASE + '/map/update'
        self.compression = 'gzip'

        # HTTP sessions
        self.session = requests.Session()
        self.session.headers['Content-Type'] = 'application/cbor'
        self.session.headers['Authorization'] = 'Bearer ' + self.token
        if self.compression == 'gzip':
            self.session.headers['Content-Encoding'] = 'gzip'
        else:
            self.session.headers['Content-Encoding'] = 'identity'

    def send_map_msg(self, typestr, msg, name):
        body = cbor.dumps({u'type': typestr, u'msg': msg, u'name': name})
        if self.compression == 'gzip':
            body = gzip.compress(bytes(body))
            print(f'  -> Sending message of size {len(body)}')
        res = self.session.post(self.map_url, body)
        return res


def create_map(arr, time_sec=0.0):
    return {
        'header': {
            'stamp': time_sec,
            'frame_id': 'darpa'
        },
        'origin': {
            'position': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 1.0
            }
        },
        'fields':
            [
                {'name': 'x', 'offset': 0, 'datatype': 7, 'count': 1},
                {'name': 'y', 'offset': 4, 'datatype': 7, 'count': 1},
                {'name': 'z', 'offset': 8, 'datatype': 7, 'count': 1},
                {'name': 'rgba', 'offset': 12, 'datatype': 6, 'count': 1},
            ],
        'point_step': 16,
        'data': arr[0].tobytes()
    }


def mapping_server0(logfile, cpr, loop=False):
    stream_id = lookup_stream_id(logfile, 'fromros.points')
    with LogReader(args.logfile, only_stream_id=stream_id) as logreader:
        for timestamp, stream, raw_data in logreader:
            arr = deserialize(raw_data)
            assert len(arr) == 1, arr  # array of arrays, but maybe it is mistake no ROS serializer side?
            if len(arr[0]) == 0:
                continue  # but maybe we should report at least one empty map?

            cloud = create_map(arr)
            print(cloud)
            if cpr is not None:
                res = cpr.send_map_msg(u'PointCloud2', msg=cloud, name=u'Skiddy')  # vs. Kloubak
                print(res)
            if not loop:
                break
            time.sleep(1.0)

def mapping_server(logfile, cpr, loop=False):
    i = 0
    while True:
        cloud = create_map(np.zeros(shape=(1, 0, 3), dtype=np.float32), time_sec=i)
        cloud['data'] = struct.pack('<fffI', i, 1, 2, 0xFF000000)
        print(cloud, len(cloud))
        res = cpr.send_map_msg(u'PointCloud2', msg=cloud, name=u'Skiddy')  # vs. Kloubak
        print(res)
        if not loop:
            break
        time.sleep(1.0)
        i += 1


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='logfile containing fromros.points')
    parser.add_argument('--token', help='Bearer communication token')#, required=True)
    parser.add_argument('--loop', help='infinite reporting loop', action='store_true')
    args = parser.parse_args()

    if args.token is not None:
        cpr = CommandPostRelay(token=args.token)
    else:
        cpr = None  # or some dummy?

    mapping_server(args.logfile, cpr, loop=args.loop)

# vim: expandtab sw=4 ts=4
