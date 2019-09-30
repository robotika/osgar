"""
  ROS (Robot Operating System) Message Parser
"""

from threading import Thread
import struct
import math
import os

from osgar.bus import BusShutdownException

ROS_MESSAGE_TYPES = {
    'std_msgs/String': '992ce8a1687cec8c8bd883ec73ca41d1',
    'geometry_msgs/Twist': '9f195f881246fdfa2798d1d3eebca84a',
    'std_msgs/Imu': '6a62c6daae103f4ff57a132d6f95cec2',
}

#### DUPLICATE with rosproxy.py !!! ####
def prefix4BytesLen(s):
    "adding ROS length"
    if type(s) == str:
        s = bytes(s, encoding='ascii')
    return struct.pack("I", len(s)) + s


def packCmdVel(speed, angularSpeed):
    return struct.pack("dddddd", speed, 0, 0, 0, 0, angularSpeed)
#########################################


def parse_imu( data ):
    # http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
    size = struct.unpack_from('<I', data)[0]
    #assert size in [324, 323], size  # expected IMU packet size (beware of variable frameIdLen - should be removed!)
    data = data[4:]
    seq, stamp, stampNsec, frameIdLen = struct.unpack("IIII", data[:16])
#    print(seq, stamp, stampNsec, frameIdLen)
#    print(data[16:16+frameIdLen])
    data = data[16+frameIdLen:]
    # variable frameIdLen:
    #    b'X1/base_link' = 12 bytes
    #    b'X3/imu_link' = 11 bytes
    assert len(data) == 296, len(data)  # fixed data size (37 * 8)
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
#    print('%d\t%f' % (stamp, math.degrees(y)))

    return linearAcceleration, (x, y, z)  # maybe later use orientation


def Xparse_image( data ):
    seq, stamp, stampNsec, frameIdLen = struct.unpack("IIII", data[:16])
#    print(seq, stamp, stampNsec, frameIdLen)
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
#    print(frame_id, timestamp_sec, timestamp_nsec)
    pos += frame_id_size
    height, width, encoding_size = struct.unpack_from('<III', data, pos)
    pos += 4 + 4 + 4
    encoding = data[pos:pos+encoding_size]
    pos += encoding_size
#    print(height, width, encoding)
    is_bigendian, step, image_arr_size = struct.unpack_from('<BII', data, pos)
    pos += 1 + 4 + 4
    if dump_filename is not None:
        with open(dump_filename, 'wb') as f:
            f.write(b'P6\n%d %d\n255\n' % (width, height))
            f.write(data[pos:pos+image_arr_size])


def parse_jpeg_image(data, dump_filename=None):
    # http://docs.ros.org/api/sensor_msgs/html/msg/CompressedImage.html
    size = struct.unpack_from('<I', data)[0]
    # http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    pos = 4
    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]
#    print(frame_id, timestamp_sec, timestamp_nsec)
    pos += frame_id_size
    encoding_size = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    encoding = data[pos:pos+encoding_size]
    pos += encoding_size
#    print(encoding, size)
    img_size = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    if dump_filename is not None:
        with open(dump_filename, 'wb') as f:
            f.write(data[pos:])
    return data[pos:]


def parse_laser(data):
    # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html
    size = struct.unpack_from('<I', data)[0]
    #assert size == 5826, size  # expected size for short lidar
    pos = 4
    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]
#    print(frame_id, timestamp_sec, timestamp_nsec)
    pos += frame_id_size
    params = struct.unpack_from('<fffffff', data, pos)
    # angle_min, angle_max, angle_increment, time_increment, scan_time
    # range_min, range_max
#    print(params)
    pos += 7*4
    ranges_size = struct.unpack_from('<I', data, pos)[0]
    assert ranges_size == 720, ranges_size
    pos += 4
    scan = struct.unpack_from('<' + 'f' * ranges_size, data, pos)
    pos += 4 * ranges_size
    intensities_size = struct.unpack_from('<I', data, pos)[0]
    assert intensities_size == 720, intensities_size
    intensities = struct.unpack_from('<' + 'f' * intensities_size, data, pos)

    # conversion to int and millimeters
    scan = [int(x*1000) if x < 65.0 else 0 for x in scan]
    return scan


def parse_odom(data):
    # http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
    size = struct.unpack_from('<I', data)[0]
    #assert size in [719, 724], size  # expected size for odometry (beware of variable header! i.e. this assert is wrong in general)
    pos = 4
    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]
#    print(frame_id, timestamp_sec, timestamp_nsec)
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
    ori = struct.unpack_from('<dddd', data, pos)
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

    q0, q1, q2, q3 = ori  # quaternion
    ax =  math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
    ay =  math.asin(2*(q0*q2-q3*q1))
    az =  math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
    return timestamp_sec, (x, y, ax)


def parse_points(data):
    # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
    size = struct.unpack_from('<I', data)[0]
    assert size == 2457728, size  # expected size for cloud points (beware of variable header! i.e. this assert is wrong in general)
    pos = 4
    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]
#    print(frame_id, timestamp_sec, timestamp_nsec)
    pos += frame_id_size
    height, width = struct.unpack_from('<II', data, pos)
    pos += 8
#    print(height, width)
    point_field_size = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    assert point_field_size == 4, point_field_size  # RGBD?
#    print(data[pos:pos + 20])
    for i in range(point_field_size):
        str_size = struct.unpack_from('<I', data, pos)[0]
        pos += 4
        assert str_size < 10, str_size
#        print(data[pos:pos + str_size])
        pos += str_size
        offset, datatype, count = struct.unpack_from('<IBI', data, pos)
#        print(offset, datatype, count)
        assert datatype == 7, datatype  # uint8 FLOAT32 = 7
        assert count == 1, count
        pos += 9

    is_bigendian, point_step, row_step = struct.unpack_from('<?II', data, pos)
    pos += 9
#    print(is_bigendian, point_step, row_step)
    arr_size = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    assert arr_size == height * width * 32, arr_size
    for i in range(height*width):
        pt = struct.unpack_from('<ffff', data, pos + i * 32)
        assert pt[3] == 0.0, pt
#        if not math.isnan(pt[0]):
#            print(pt[:3])
#        assert str(pt) == str((nan, nan, nan, 0.0)), (i, pt)

    pos += row_step * height
    is_dense = struct.unpack_from('<?', data, pos)[0]
    pos += 1
#    print(is_dense)
    assert pos == len(data), (pos, len(data))


class ROSMsgParser(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self._buf = b''

        self.topic_type = config.get('topic_type')
        self.timestamp_sec = None

        # initial message contains structure
        self.header = None
        self.count = 0
        self.downsample = config.get('downsample', 1)

        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0

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

    def slot_raw(self, timestamp, data):
#        self._buf += data
#        packet = self.get_packet()
#        if packet is None:
#            return
#        if self.header is None:
#            self.header = packet
#            return
#        self.count += 1
#        if self.count % self.downsample != 0:
#            return
        packet = data  # ZMQ hack
        # TODO parse properly header "frame ID"
        if b'X2/base_link/camera_front' in packet:  #self.topic_type == 'sensor_msgs/CompressedImage':
            self.bus.publish('image', parse_jpeg_image(packet))
        elif b'X2/base_link/front_laser' in packet:  #self.topic_type == 'sensor_msgs/LaserScan':
            self.count += 1
            if self.count % self.downsample != 0:
                return
            self.bus.publish('scan', parse_laser(packet))
        elif b'X2/odom' in packet:  #self.topic_type == 'nav_msgs/Odometry':
            prev = self.timestamp_sec
            self.timestamp_sec, (x, y, heading) = parse_odom(packet)
            self.bus.publish('pose2d', [round(x*1000),
                                        round(y*1000),
                                        round(math.degrees(heading)*100)])
            if prev != self.timestamp_sec:
                self.bus.publish('sim_time_sec', self.timestamp_sec)
        elif b'X2/base_link/imu_sensor' in packet:  # self.topic_type == 'std_msgs/Imu':
            acc, rot = parse_imu(packet)
            self.bus.publish('rot', [round(math.degrees(angle)*100) 
                                     for angle in rot])
            self.bus.publish('acc', [round(x * 1000) for x in acc])

    def slot_tick(self, timestamp, data):
        #cmd = prefix4BytesLen(packCmdVel(self.desired_speed, self.desired_angular_speed))
        cmd = b'cmd_vel %f %f' % (self.desired_speed, self.desired_angular_speed)
        self.bus.publish('cmd_vel', cmd)

    def slot_desired_speed(self, timestamp, data):
        self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)

    def slot_stdout(self, timestamp, data):
        cmd = b'stdout ' + data  # redirect to ROS_INFO
        self.bus.publish('cmd_vel', cmd)

    def run(self):
        try:
            while True:
                timestamp, channel, data = self.bus.listen()
                if channel == 'raw':
                    self.slot_raw(timestamp, data)
                elif channel == 'tick':
                    self.slot_tick(timestamp, data)
                elif channel == 'desired_speed':
                    self.slot_desired_speed(timestamp, data)
                elif channel == 'stdout':
                    self.slot_stdout(timestamp, data)
                else:
                    assert False, channel  # unsupported input channel
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
