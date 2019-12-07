#!/usr/bin/python
"""
  Statistics on ROSbag data
  usage:
       ./rosbag.py <input file>
"""
# based on format specification at
#   http://wiki.ros.org/Bags/Format/2.0
import os
import struct


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


def read_rosbag_gen(filename):
    with open(filename, 'rb') as f:
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
            if 'topic' in header_dict:
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


def extract_log(filename):
    out_name = os.path.join(os.path.dirname(filename), 'tmp.log')
    with open(out_name, 'wb') as f:
        for header_dict, data in read_rosbag_gen(filename):
            op = header_dict['op']
            conn  = header_dict.get('conn')
            if conn == 0:
                if op == OP_MESSAGE_DATA:
                    d = parse_string(data)
                    if b'Hello' not in d:
                        f.write(d)
                    else:
                        print(d, len(d))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='JPEG filename')
    parser.add_argument('-v', '--verbose', help='verbose mode', action='store_true')
    args = parser.parse_args()

    extract_log(args.filename)


# vim: expandtab sw=4 ts=4 

