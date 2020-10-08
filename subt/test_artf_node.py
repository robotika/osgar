import unittest

import numpy as np

from subt.artf_node import result2report, check_results


class ArtifactDetectorDNNTest(unittest.TestCase):

    def test_result2report(self):
        result = [('backpack', [(60, 180, 0.9785775), (72, 180, 0.9795098), (60, 184, 0.97716093), (72, 184, 0.9782014)])]
        row = [5000]*640
        depth = np.array([row]*360, dtype=np.uint16)
        self.assertEqual(result2report(result, depth), ['TYPE_BACKPACK', 2754, 5000])

        row = list(range(640))
        depth = np.array([row]*360, dtype=np.uint16)
        self.assertEqual(result2report(result, depth), ['TYPE_BACKPACK', 2754, 66])

        result2 = [('rope', [(400, 180, 0.9785775)])]
        self.assertEqual(result2report(result2, depth), ['TYPE_ROPE', -868, 400])

    def test_out_of_range(self):
        result = [('backpack', [(60, 180, 0.9785775), (72, 180, 0.9795098), (60, 184, 0.97716093), (72, 184, 0.9782014)])]
        row = [0xFFFF]*640
        depth = np.array([row]*360, dtype=np.uint16)
        self.assertEqual(result2report(result, depth), None)

    def test_check_reults(self):
        result = [('backpack', [(60, 180, 0.9785775), (72, 180, 0.9795098)]),
                  ('backpack', [(260, 10, 0.9785775), (261, 10, 0.9795098)]),  # out of bbox, false detection
                  ('rope', [(400, 100, 0.9785775), (401, 100, 0.9795098)])
                  ]
        result_cv = [['backpack', 0.99990773, [50, 150, 200, 250]],
                     ['rope', 0.99650773, [350, 90, 500, 250]]
                     ]
        expected_result = [('backpack', [(60, 180, 0.9785775), (72, 180, 0.9795098)]),
                           ('rope', [(400, 100, 0.9785775), (401, 100, 0.9795098)])
                          ]
        checked_result = check_results(result, result_cv)
        self.assertEqual(checked_result, expected_result)

    def test_avoid_double_detection(self):
        result = [('backpack', [(100, 200, 0.9785775), (101, 200, 0.9795098)])]
        result_cv = [['backpack', 0.99990773, [50, 150, 200, 250]], ['backpack', 0.99990773, [60, 150, 210, 250]]]
        checked_result = check_results(result, result_cv)
        self.assertEqual(checked_result, result)

    def test_merge_two_results(self):
        result = [('backpack', [(100, 200, 0.9785775), (101, 200, 0.9795098)]),
                  ('backpack', [(102, 200, 0.9785775), (103, 200, 0.9795098)])
                  ]
        result_cv = [['backpack', 0.99990773, [50, 150, 200, 250]]]
        expected_result = [('backpack', [(100, 200, 0.9785775), (101, 200, 0.9795098),
                                         (102, 200, 0.9785775), (103, 200, 0.9795098)])
                           ]
        checked_result = check_results(result, result_cv)
        self.assertEqual(checked_result, expected_result)


if __name__ == "__main__":
    # run specific test based on recorded stdout
    import argparse
    from datetime import timedelta
    from osgar.lib.serialize import deserialize
    from osgar.logger import LogReader, lookup_stream_id, lookup_stream_names
    from ast import literal_eval

    parser = argparse.ArgumentParser(description='Test 3D reports')
    parser.add_argument('logfile', help='OSGAR logfile')
    parser.add_argument('--time-limit-sec', '-t', help='cut time in seconds', type=float)
    args = parser.parse_args()

    names = lookup_stream_names(args.logfile)
    stdout_stream_id = lookup_stream_id(args.logfile, 'detector.stdout')
    streams = [stdout_stream_id]
    if 'detector.debug_depth' in names:
        depth_stream_id = lookup_stream_id(args.logfile, 'detector.debug_depth')
        streams.append(depth_stream_id)
    else:
        depth_stream_id = None

    # dummy depth for loder logfiles
    row = [5000] * 640
    depth = np.array([row] * 360, dtype=np.uint16)
    fx = 554.25469
    with LogReader(args.logfile,
                   only_stream_id=streams) as logreader:
        for time, stream, msg_data in logreader:
            if args.time_limit_sec is not None:
                if time.total_seconds() > args.time_limit_sec:
                    break
            data = deserialize(msg_data)
            if stream == depth_stream_id:
                depth = data
                continue

            assert stream == stdout_stream_id
            print(time, data)
            try:
                arr = literal_eval(data)  # well the new stdout is not suitable for this :(
            except:
                arr = None

            if arr is not None:
                ret = result2report(arr, depth, fx)
                print(ret)


# vim: expandtab sw=4 ts=4
