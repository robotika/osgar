#!/usr/bin/python
"""
  Extract OSGAR logfile from SubT ROS Bag (topic /robot_data)
"""
# based on format specification at
#   http://wiki.ros.org/Bags/Format/2.0
import os
import struct
import tarfile

MAX_RECORD_SIZE = 100000000

OP_MESSAGE_DATA = 0x2
OP_INDEX_DATA = 0x4
OP_CHUNK = 0x5
OP_CHUNK_INFO = 0x6
OP_CONNECTION = 0x7

IMAGE_TOPIC_ID = 59  # /usb_cam/image_raw


def parse_header(header):
    index = 0
    ret = {}
    while index < len(header):
        size = struct.unpack_from('<I', header, index)[0]
        assert size > 0, size
        record = header[index + 4 : index + 4 + size]
        assert b'=' in record, record
        key = record.split(b'=')[0].decode('ascii')
        value = record[len(key)+1:]  # potentially binary blob
        ret[key] = value  # by default keep original binary data
        if key == 'op':
            assert len(value) == 1, len(value)
            # bag header, chunk, index, connection, chunk info
            assert value[0] in [0x2, 0x3, 0x5, 0x4, 0x7, 0x6], value[0]
            ret[key] = value[0]
        elif key == 'conn':
            assert len(value) == 4, len(value)
            ret[key] = struct.unpack('<I', value)[0]
        elif key == 'count':
            assert len(value) == 4, len(value)
            ret[key] = struct.unpack('<I', value)[0]
        elif key == 'topic':
            # actually expected only in OP=0x7 (connection)
            pass
        elif key == 'compression':
            assert value == b'none', value  # note, that b'bz2' is also supported
        elif key == 'time':
            assert len(value) == 8, len(value)
            ret[key] = struct.unpack('<II', value)
        index += 4 + size
    assert index == len(header), (index, len(header))
    return ret


def handle_message_data(index_header_dict, data):
    # verify index header and split block data into new header and data
    header_size = struct.unpack_from('<I', data)[0]
    header_dict = parse_header(data[4:4+header_size])
    size = struct.unpack_from('<I', data, 4+header_size)[0]
    return header_dict, data[4+header_size:4+header_size+4+size]


def read_rosbag_fd_gen(f, verbose=False):
    """Read ROS Bag with already opened file descriptor f"""
    header = f.read(13)
    assert header == b'#ROSBAG V2.0\n', header

    header_len = f.read(4)
    chunk = None
    while len(header_len) == 4:
        size = struct.unpack('<I', header_len)[0]
        assert size < MAX_RECORD_SIZE, size
        header = f.read(size)
        assert len(header) == size, (len(header), size)
        header_dict = parse_header(header)
        if 'topic' in header_dict and verbose:
            print('topic', header_dict['conn'], header_dict['topic'])

        data_len = f.read(4)
        assert len(data_len) == 4, len(data_len)
        size = struct.unpack('<I', data_len)[0]
        assert size < MAX_RECORD_SIZE, size
        data = f.read(size)
        #assert len(data) == size, (len(data), size)
        if len(data) != size:
            print("Error", (len(data), size))
            return
        op = header_dict['op']
        if op == OP_CHUNK:
            chunk = data
        elif op == OP_INDEX_DATA:
            header_count = header_dict['count']
            assert chunk is not None  # first data then indexes
            # note, that one "chunk" can be shared by several "indexes"
            assert size == 12 * header_count, size
            for t, offset in struct.iter_unpack('<QI', data):
                assert offset < len(chunk)
                yield handle_message_data(header_dict, chunk[offset:])
        else:
            yield header_dict, data_len + data
        header_len = f.read(4)


def read_rosbag_gen(filename):
    with open(filename, 'rb') as f:
        for data in read_rosbag_fd_gen(f):
            yield  data


def parse_raw_image(data, dump_filename=None):
    # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
    size = struct.unpack_from('<I', data)[0]
    assert size == 2764848, size  # expected size for raw image during experiment
    # http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    pos = 4
    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]
    print(frame_id, timestamp_sec, timestamp_nsec)
    pos += frame_id_size
    height, width, encoding_size = struct.unpack_from('<III', data, pos)
    pos += 4 + 4 + 4
    encoding = data[pos:pos+encoding_size]
    pos += encoding_size
    print(height, width, encoding)
    is_bigendian, step, image_arr_size = struct.unpack_from('<BII', data, pos)
    pos += 1 + 4 + 4
    if dump_filename is not None:
        with open(dump_filename, 'wb') as f:
            f.write(b'P6\n%d %d\n255\n' % (width, height))
            f.write(data[pos:pos+image_arr_size])


def parse_string(data, dump_filename=None):
    # http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    size = struct.unpack_from('<I', data)[0]
#    print(size)
    pos = 4
    str_size = struct.unpack_from('<I', data, pos)[0]
#    print(size, str_size)
#    assert size == str_size + 4, (size, str_size)
    pos += 4
    return data[pos:]


def extract_log(gen, out_name, append=False, verbose=False):
    with open(out_name, 'ab' if append else 'wb') as f:
        for header_dict, data in gen:
            op = header_dict['op']
            conn  = header_dict.get('conn')
            if conn == 0:
                if op == OP_MESSAGE_DATA:
                    d = parse_string(data)
                    if b'Hello' not in d:
                        f.write(d)
                    elif verbose:
                        print(d, len(d))


def process_tar(filepath, all, verbose):
        # In: "ver41p3-91c332d3-2066-466c-a9b9-e3418bbeb0a9-A10F900L.tar"
        # Out: "aws-ver41p3-A10F900L.log"
        basename_without_ext = os.path.basename(filepath).split('.', 1)[0]
        s = basename_without_ext.split('-')
        robot_name = s[-2] if s[-1] == "commsbridge" else s[-1]
        name = robot_name + ".log"
        letter = robot_name[0]
        out_name = os.path.join(os.path.dirname(filepath), name)
        robot_data_0_processed = False
        robot_data_1_processed = False
        with tarfile.open(filepath, "r") as tar:
            for member in tar.getmembers():
                if member.name.startswith('robot_data_'):
                    # new names contain hash of simulation and robot name, i.e.
                    # robot_data_affcd00f-3ccf-4ebc-9620-d41635d7044e-r-1_A600L_0.bag
                    # robot_data_affcd00f-3ccf-4ebc-9620-d41635d7044e-r-1_A600L_1.bag
                    #   OR
                    # robot_data_affcd00f-3ccf-4ebc-9620-d41635d7044e-r-1_C300R_0.bag.active
                    if '_0.bag' in member.name:
                        print(member.name, "->", name)
                        f = tar.extractfile(member)
                        extract_log(read_rosbag_fd_gen(f, verbose=verbose), out_name, verbose=verbose)
                        robot_data_0_processed = True
                    elif '_1.bag' in member.name:
                        if not all:
                            print(member.name, "->", "SKIPPED")
                            continue
                        print(member.name, "->", name)
                        assert robot_data_0_processed
                        f = tar.extractfile(member)
                        extract_log(read_rosbag_fd_gen(f, verbose=verbose), out_name, append=True, verbose=verbose)
                        robot_data_1_processed = True
                    elif '_2.bag' in member.name:
                        if not all:
                            print(member.name, "->", "SKIPPED")
                            continue
                        print(member.name, "->", name)
                        assert robot_data_0_processed
                        assert robot_data_1_processed
                        f = tar.extractfile(member)
                        extract_log(read_rosbag_fd_gen(f, verbose=verbose), out_name, append=True, verbose=verbose)
                    else:
                        assert False, member.name  # unexpected "robot_data_" rosbag name
                elif member.name.endswith('rosout.log'):
                    rosout_name = letter + '-rosout.log'
                    print(member.name, "->", rosout_name)
                    with open(os.path.join(os.path.dirname(filepath), rosout_name), 'wb') as f:
                        f.write(tar.extractfile(member).read())
                elif (member.name in ['server_console.log'] or member.name.endswith('.yml') or
                      member.name.startswith('subt_urban_') or member.name.startswith('subt_cave_') or
                      (member.name.startswith('state.tlog') and all)):
                    print(member.name)
                    with open(os.path.join(os.path.dirname(filepath), member.name.replace(':', '_')), 'wb') as f:
                        f.write(tar.extractfile(member).read())


def main():
    import argparse
    import pathlib
    import sys

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filenames', help='ROS bag file(s) or tar(.gz)', nargs='*')
    #parser.add_argument('-a', '--all', help='extract all robot_data* files', action='store_true')
    parser.add_argument('--debug', help='debug output', action='store_true')
    args = parser.parse_args()

    if args.filenames == []:
        for p in pathlib.Path('.').iterdir():
            if p.suffixes == ['.tar', '.gz'] or p.suffixes == ['.tar']:
                args.filenames.append(str(p))
        if len(args.filenames) == 0:
            sys.exit("no logfiles found in current directory")
        args.filenames.sort()

    for filename in args.filenames:
        print("processing:", filename)
        if filename.endswith('.tar') or filename.endswith('.tar.gz'):
            process_tar(filename, all=True, verbose=args.debug)
        else:
            out_name = os.path.join(os.path.dirname(filename), 'tmp.log')
            extract_log(read_rosbag_gen(filename), out_name, verbose=args.debug)


if __name__ == "__main__":
    main()
# vim: expandtab sw=4 ts=4 

