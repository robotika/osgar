#!/usr/bin/python
"""
  Conversion utility from socket bin files to pcap format
  usage:
       ./bin2pcap.py <input file(s)> <output directory>
"""
import sys
import os
from binascii import unhexlify

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


def bin2pcap(filename, output_dir):
    new_name = os.path.join(output_dir, os.path.split(filename)[1])
    assert new_name.endswith('.bin'), new_name
    new_name = new_name[:-4] + '.pcap'
    print(new_name)
    f = open(filename, 'rb')
    out = open(new_name, 'wb')
    out.write(unhexlify(FILE_HEADER))
    while True:
        packet = f.read(1206)
        if len(packet) == 0:
            break  # EOF
        assert len(packet) == 1206, len(packet)
        out.write(unhexlify(PACKET_HEADER))  # TODO revise timestamps
        out.write(unhexlify(IP_HEADER))
        out.write(packet)


def velodyne_bin_file(filename):
    for line in open(filename):
        if line.startswith('velodyne:'):
            return os.path.join(os.path.split(filename)[0], 
                                line.split('/')[-1].strip())
    return None

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)

    output_dir = sys.argv[-1]
    for filename in sys.argv[1:-1]:
        if 'meta_' in filename:
            filename = velodyne_bin_file(filename)
        if filename:
            bin2pcap(filename, output_dir)

# vim: expandtab sw=4 ts=4 

