"""
  ROS (Robot Operating System) Message Parser
"""

from threading import Thread
import struct
import math


from osgar.bus import BusShutdownException

ROS_MESSAGE_TYPES = {
    'std_msgs/String': '992ce8a1687cec8c8bd883ec73ca41d1',
    'geometry_msgs/Twist': '9f195f881246fdfa2798d1d3eebca84a',
    'std_msgs/Imu': '6a62c6daae103f4ff57a132d6f95cec2',
}


def prefix4BytesLen(s):
    "adding ROS length"
    if type(s) == str:
        s = bytes(s, encoding='ascii')
    return struct.pack("I", len(s)) + s


def parse_imu( data ):
    seq, stamp, stampNsec, frameIdLen = struct.unpack("IIII", data[:16])
#    print(seq, stamp, stampNsec, frameIdLen)
    data = data[16+frameIdLen:]
    orientation = struct.unpack("dddd", data[:4*8])
    data = data[4*8+9*8:] # skip covariance matrix
    angularVelocity = struct.unpack("ddd", data[:3*8])
    data = data[3*8+9*8:] # skip velocity covariance
    linearAcceleration = struct.unpack("ddd", data[:3*8])
#    print('%d\t%f' % (stamp, sum([x*x for x in linearAcceleration])))
    data = data[3*8+9*8:] # skip velocity covariance
    assert len(data) == 0, len(data)

    q0, q1, q2, q3 = orientation  # quaternion
    x =  math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
    y =  math.asin(2*(q0*q2-q3*q1))
    z =  math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
    print('%d\t%f' % (stamp, math.degrees(y)))

    return orientation


def Xparse_image( data ):
    seq, stamp, stampNsec, frameIdLen = struct.unpack("IIII", data[:16])
    print(seq, stamp, stampNsec, frameIdLen)
    data = data[16+frameIdLen:]

# from rosbag.py
def parse_raw_image(data, dump_filename=None):
    # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
    size = struct.unpack_from('<I', data)[0]
    assert size == 230467, size  # expected size for raw image during experiment
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


def parse_jpeg_image(data, dump_filename=None):
    # http://docs.ros.org/api/sensor_msgs/html/msg/CompressedImage.html
    size = struct.unpack_from('<I', data)[0]
    assert size < 230467, size  # expected size for raw image during experiment
    # http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    pos = 4
    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]
    print(frame_id, timestamp_sec, timestamp_nsec)
    pos += frame_id_size
    encoding_size = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    encoding = data[pos:pos+encoding_size]
    pos += encoding_size
    print(encoding, size)
    img_size = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    if dump_filename is not None:
        with open(dump_filename, 'wb') as f:
            f.write(data[pos:])
    return data[pos:]


def parse_laser(data):
    # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html
    size = struct.unpack_from('<I', data)[0]
    assert size == 5826, size  # expected size for short lidar
    pos = 4
    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]
    print(frame_id, timestamp_sec, timestamp_nsec)
    pos += frame_id_size
    params = struct.unpack_from('<fffffff', data, pos)
    # angle_min, angle_max, angle_increment, time_increment, scan_time
    # range_min, range_max
    print(params)
    pos += 7*4
    ranges_size = struct.unpack_from('<I', data, pos)[0]
    assert ranges_size == 720, ranges_size
    pos += 4
    scan = struct.unpack_from('<' + 'f' * ranges_size, data, pos)
    pos += 4 * ranges_size
    intensities_size = struct.unpack_from('<I', data, pos)[0]
    assert intensities_size == 720, intensities_size
    intensities = struct.unpack_from('<' + 'f' * intensities_size, data, pos)
    return scan  # TODO conversion to integers??


def parse_odom(data):
    # http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
    size = struct.unpack_from('<I', data)[0]
    assert size == 719, size  # expected size for short lidar
    pos = 4
    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]
    print(frame_id, timestamp_sec, timestamp_nsec)
    pos += frame_id_size
    child_frame_id = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    #print(data[pos:pos + child_frame_id])
    pos += child_frame_id
    # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html
    # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html
    # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Point.html
    # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html
    x, y, z = struct.unpack_from('<ddd', data, pos)
    pos += 3 * 8
    quat = struct.unpack_from('<dddd', data, pos)
    pos += 4 * 8
    # float64[36] covariance
    pos += 36 * 8
    #print(x, y, z)

    # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html
    linear = struct.unpack_from('<ddd', data, pos)
    pos += 3 * 8
    angular = struct.unpack_from('<ddd', data, pos)
    pos += 3 * 8
    # float64[36] covariance
    pos += 36 * 8
    assert pos == len(data), (pos, len(data))
    return (x, y, 0.0)  # TODO heading from quaternion


class ROSMsgParser(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self._buf = b''

        self.topic_type = config.get('topic_type')

    def get_packet(self):
        data = self._buf
        if len(data) < 4:
            return None
        size = struct.unpack_from('<I', data, 0)[0]
#        print(size, len(self._buf))
        assert size > 0, size
        size += 4  # the length prefix
        if len(data) < size:
            return None
        ret, self._buf = data[:size], data[size:]        
        return ret

    def run(self):
        try:
            header = None
            # initial message contains structure
            while True:
                timestamp, channel, data = self.bus.listen()
                self._buf += data
                packet = self.get_packet()
                if packet is not None:
                    if header is None:
                        header = packet
                    elif self.topic_type == 'sensor_msgs/CompressedImage':
                        self.bus.publish('image', parse_jpeg_image(packet))
                    elif self.topic_type == 'sensor_msgs/LaserScan':
                        self.bus.publish('scan', parse_laser(packet))
                    elif self.topic_type == 'nav_msgs/Odometry':
                        x, y, heading = parse_odom(packet)
                        self.bus.publish('pose2d', [round(x*1000),
                                    round(y*1000),
                                    round(math.degrees(heading)*100)])
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
