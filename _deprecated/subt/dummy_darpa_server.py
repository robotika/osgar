#!/usr/bin/env python
"""
  Dummy DARPA scoring server
"""

import os
import sys
import csv
import math
import json
import logging
from collections import defaultdict
from http.server import BaseHTTPRequestHandler, HTTPServer
from mimetypes import guess_type


g_logger = logging.getLogger(__name__)


def dist3d(xyz, xyz2):
    return math.sqrt(sum([(a-b)**2 for a, b in zip(xyz, xyz2)]))


class GameLogic:
    def __init__(self, filename):
        self.score = 0
        self.artf = defaultdict(list)
        with open(filename) as csvfile:
            reader = csv.reader(csvfile)
            for raw in reader:
                if 'artf' in raw:
                    continue
                # artifact name, x, y, z
                artf = raw[0]
                self.artf[artf].append(tuple(float(x) for x in raw[1:]))

    def report_artf(self, artf, xyz):
        if artf in self.artf:
            best = None
            for artf_xyz in self.artf[artf]:
                if best is None or dist3d(xyz, artf_xyz) < best[0]:
                    best = dist3d(xyz, artf_xyz), artf_xyz
            if best is not None and best[0] < 5.0:  # DARPA 5m limit
                self.artf[artf].remove(best[1])
                self.score += 1
                return True
        return False


class MyHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        g_logger.info(f"GET: {self.path}")
        s = self.path.split('/')
        g_logger.info(str(s))
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"score":%d,"remaining_reports":97,"current_team":"robotika","run_clock":1502.8}' % self.server.game_logic.score)

    def do_POST(self):
        g_logger.info(f"POST: {self.path}")
        s = self.path.split('/')
        assert self.headers['Content-Type'] == 'application/json', self.headers['Content-Type']
        assert 'artifact_reports' in s, s
        size = int(self.headers['Content-Length'])
        data = self.rfile.read(size)
        g_logger.info(f'DATA {data}')
        d = json.loads(data)
        self.server.game_logic.report_artf(d['type'], (d['x'], d['y'], d['z']))
        self.send_response(201)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"url":"http://10.100.2.200:8000/api/reports/3/","id":3,"x":1.0,"y":2.0,"z":4.0,"type":"Cell Phone","submitted_datetime":"2020-02-18T22:40:05.009145+00:00","run_clock":1505.0,"team":"robotika","run":"0.0.2","report_status":"scored","score_change":1}')


def main():
    import argparse

    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
        datefmt='%Y-%m-%d %H:%M',
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='CSV file with header "artf, x, y, z"')
    args = parser.parse_args()
    try:
        server = HTTPServer(('',8888), MyHandler)
        server.game_logic = GameLogic(args.filename)
        print('started httpserver...')
        server.serve_forever()
    except KeyboardInterrupt:
        print('keyboard interrupt')
        server.socket.close()

if __name__ == '__main__':
    main()

# vim: expandtab sw=4 ts=4 

