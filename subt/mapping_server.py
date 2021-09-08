"""
   Mapping server for Teambase to report maps to DARPA
   for SubT finals.
"""
import requests
import time
import cbor
import gzip

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


def create_empty_map():
    return {
        'fields':
            [
                {'name': 'x', 'offset': 0, 'datatype': 7, 'count': 1},
                {'name': 'y', 'offset': 4, 'datatype': 7, 'count': 1},
                {'name': 'z', 'offset': 8, 'datatype': 7, 'count': 1},
            ],
        'point_step': 12,
        'data': b''
    }


def mapping_server(token, cloud, loop=False):
    cpr = CommandPostRelay(token=token)
    while True:
        res = cpr.send_map_msg(u'PointCloud2', msg=cloud, name=u'Skiddy')  # vs. Kloubak
        print(res)
        if not loop:
            break
        time.sleep(1.0)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--token', help='Bearer communication token', required=True)
    parser.add_argument('--loop', help='infinite reporting loop', action='store_true')
    args = parser.parse_args()

    mapping_server(args.token, create_empty_map(), loop=args.loop)

# vim: expandtab sw=4 ts=4
