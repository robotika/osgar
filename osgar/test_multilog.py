import unittest
import os
import tempfile
import json
import pathlib
import datetime

from osgar.logger import (
    LogWriter, LogReader, LogReaderEx, MultiLogReader,
    lookup_stream_names, lookup_stream_id, lookup_config, INFO_STREAM_ID
)
from osgar.lib.serialize import serialize, deserialize

class MultiLogTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.dir_path = self.temp_dir.name

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_multi_log_functionality(self):
        # 1. Create two sub-logs with custom start times
        start_m03 = datetime.datetime(2026, 8, 1, 10, 50, 0, tzinfo=datetime.timezone.utc)
        start_pat = datetime.datetime(2026, 8, 1, 10, 50, 5, tzinfo=datetime.timezone.utc)

        file_m03 = os.path.join(self.dir_path, "m03.log")
        file_pat = os.path.join(self.dir_path, "pat.log")

        # Create m03 log
        with LogWriter(filename=file_m03, start_time=start_m03) as writer:
            # Write a config dict to stream 0
            writer.write(stream_id=INFO_STREAM_ID, data=bytes(str({"m03_cfg": True}), encoding='ascii'), dt=datetime.timedelta())
            # Register "pose" stream -> stream ID 1
            writer.register("pose", dt=datetime.timedelta())
            # Write data packets
            writer.write(1, serialize("pos1"), dt=datetime.timedelta(seconds=1.0))
            writer.write(1, serialize("pos2"), dt=datetime.timedelta(seconds=3.0))

        # Create pat log
        with LogWriter(filename=file_pat, start_time=start_pat) as writer:
            # Write a config dict to stream 0
            writer.write(stream_id=INFO_STREAM_ID, data=bytes(str({"pat_cfg": 42}), encoding='ascii'), dt=datetime.timedelta())
            # Register "speed" stream -> stream ID 1
            writer.register("speed", dt=datetime.timedelta())
            # Write data packets
            writer.write(1, serialize(10), dt=datetime.timedelta(seconds=2.0))
            writer.write(1, serialize(20), dt=datetime.timedelta(seconds=4.0))

        # 2. Define Multi-Log configuration dictionary and JSON file
        config_dict = {
            "m03": {
                "file": "m03.log",
                "offset_sec": -1.0
            },
            "pat": {
                "file": "pat.log"
            }
        }
        
        config_file_path = os.path.join(self.dir_path, "multilog_config.json")
        with open(config_file_path, "w", encoding="utf-8") as f:
            json.dump(config_dict, f)

        # 3. Test lookup_stream_names
        # Note: lookup_stream_names must correctly handle absolute/relative paths from the config file location
        # Since files are "m03.log" and "pat.log" next to "multilog_config.json", it should resolve them.
        names = lookup_stream_names(config_file_path)
        self.assertEqual(names, ["m03.pose", "pat.speed"])

        # Test lookup_stream_id
        self.assertEqual(lookup_stream_id(config_file_path, "m03.pose"), 1)
        self.assertEqual(lookup_stream_id(config_file_path, "pat.speed"), 2)

        # Test lookup_config
        cfgs = lookup_config(config_file_path)
        self.assertEqual(cfgs, {
            "m03": {"m03_cfg": True},
            "pat": {"pat_cfg": 42}
        })

        # 4. Test reading from MultiLogReader directly
        with MultiLogReader(config_file_path) as reader:
            # T_ref = min(start_m03 + -1.0, start_pat + 0.0)
            # T_ref = min(10:49:59, 10:50:05) = 10:49:59
            # delta_m03 = start_m03 + -1.0 - T_ref = 0
            # delta_pat = start_pat - T_ref = 6 seconds
            # m03 packet 1: local 1.0 -> global 1.0
            # m03 packet 2: local 3.0 -> global 3.0
            # pat packet 1: local 2.0 -> global 8.0
            # pat packet 2: local 4.0 -> global 10.0
            expected_start_time = datetime.datetime(2026, 8, 1, 10, 49, 59, tzinfo=datetime.timezone.utc)
            self.assertEqual(reader.start_time, expected_start_time)

            packets = list(reader)
            self.assertEqual(len(packets), 4)

            # Assert order and correct virtual IDs
            self.assertEqual(packets[0][0], datetime.timedelta(seconds=1.0))
            self.assertEqual(packets[0][1], 1) # m03.pose virtual ID is 1
            self.assertEqual(deserialize(packets[0][2]), "pos1")

            self.assertEqual(packets[1][0], datetime.timedelta(seconds=3.0))
            self.assertEqual(packets[1][1], 1)
            self.assertEqual(deserialize(packets[1][2]), "pos2")

            self.assertEqual(packets[2][0], datetime.timedelta(seconds=8.0))
            self.assertEqual(packets[2][1], 2) # pat.speed virtual ID is 2
            self.assertEqual(deserialize(packets[2][2]), 10)

            self.assertEqual(packets[3][0], datetime.timedelta(seconds=10.0))
            self.assertEqual(packets[3][1], 2)
            self.assertEqual(deserialize(packets[3][2]), 20)

        # 5. Test reading from LogReaderEx
        with LogReaderEx(config_file_path) as reader:
            self.assertEqual(reader.start_time, expected_start_time)
            decoded_packets = list(reader)
            self.assertEqual(len(decoded_packets), 4)
            self.assertEqual(decoded_packets[0], (datetime.timedelta(seconds=1.0), "m03.pose", "pos1"))
            self.assertEqual(decoded_packets[1], (datetime.timedelta(seconds=3.0), "m03.pose", "pos2"))
            self.assertEqual(decoded_packets[2], (datetime.timedelta(seconds=8.0), "pat.speed", 10))
            self.assertEqual(decoded_packets[3], (datetime.timedelta(seconds=10.0), "pat.speed", 20))

        # 6. Test filtering by names under LogReaderEx
        with LogReaderEx(config_file_path, names=["pat.speed"]) as reader:
            filtered_packets = list(reader)
            self.assertEqual(len(filtered_packets), 2)
            self.assertEqual(filtered_packets[0], (datetime.timedelta(seconds=8.0), "pat.speed", 10))
            self.assertEqual(filtered_packets[1], (datetime.timedelta(seconds=10.0), "pat.speed", 20))

        # 7. Test calculate_stat with Multi-Log config file
        from osgar.logger import calculate_stat
        stat_names, stat_sizes, stat_counts, stat_timestamp = calculate_stat(config_file_path)
        self.assertEqual(stat_names, ["sys", "m03.pose", "pat.speed"])
        self.assertEqual(stat_counts[0], 0)  # sys has 0 packets
        self.assertEqual(stat_counts[1], 2)  # m03.pose has 2 packets
        self.assertEqual(stat_counts[2], 2)  # pat.speed has 2 packets
        self.assertEqual(stat_timestamp, datetime.timedelta(seconds=10.0))

    def test_cli_main(self):
        import sys
        from unittest.mock import patch
        from io import StringIO
        from osgar.logger import main

        # Create two sub-logs and a json config file
        start_m03 = datetime.datetime(2026, 8, 1, 10, 50, 0, tzinfo=datetime.timezone.utc)
        file_m03 = os.path.join(self.dir_path, "cli_m03.log")
        with LogWriter(filename=file_m03, start_time=start_m03) as writer:
            writer.register("pose", dt=datetime.timedelta())
            writer.write(1, serialize("pos1"), dt=datetime.timedelta(seconds=1.0))

        config_dict = {
            "m03": {
                "file": "cli_m03.log"
            }
        }
        config_file_path = os.path.join(self.dir_path, "cli_multilog_config.json")
        with open(config_file_path, "w", encoding="utf-8") as f:
            json.dump(config_dict, f)

        test_args = ["logger", config_file_path]
        with patch.object(sys, 'argv', test_args):
            with patch('sys.stdout', new=StringIO()) as fake_out:
                with self.assertRaises(SystemExit) as cm:
                    main()
                self.assertTrue(cm.exception.code in (0, None))
                output = fake_out.getvalue()
                self.assertIn("m03.pose", output)
                self.assertIn("Total time", output)

if __name__ == "__main__":
    unittest.main()
