#!/usr/bin/python
"""
  Conversion utility from log files to pcap format
  usage:
       ./log2pcap.py <input file(s)> <output directory>
"""
import sys
import os
from binascii import unhexlify

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize

# Notes:
#   Source https://github.com/kisom/pypcapfile was used for parsing original
# pcap files. There is 24byte header and each packet has 16byte header with
# timestamp and packet length followed by IP/UDP header and data.
#
# The data can be then viewer via VeloView:
#    http://www.paraview.org/Wiki/VeloView


FILE_HEADER = 'd4c3b2a1020004000000000000000000ffff000001000000'
PACKET_HEADER = '715fb15517bb0a00e0040000e0040000'
IP_HEADER = 'ffffffffffff6076880000000800450004d200004000ff11b4aac0a801c8ffffffff0940094004be0000'


def log2pcap(input_filepath, output_dir):
    """
       Extract from logfile Velodyne UPD packets and save them in 'pcap' format
       undestandable by Wireshark and VeloView.
    """
    output_filepath = os.path.join(output_dir, os.path.basename(input_filepath))
    assert output_filepath.endswith('.log'), output_filepath
    output_filepath = output_filepath[:-4] + '.pcap'
    print(output_filepath)

    only_stream = lookup_stream_id(input_filepath, 'velodyne.raw')
    with LogReader(input_filepath, only_stream_id=only_stream) as log, open(output_filepath, 'wb') as out:
        out.write(unhexlify(FILE_HEADER))
        for timestamp, stream_id, data in log:
            packet = deserialize(data)
            assert len(packet) == 1206, len(packet)

            out.write(unhexlify(PACKET_HEADER))  # TODO revise timestamps
            out.write(unhexlify(IP_HEADER))
            out.write(packet)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)

    output_dir = sys.argv[-1]
    for filename in sys.argv[1:-1]:
        log2pcap(filename, output_dir)

# vim: expandtab sw=4 ts=4 

