"""
  ROS (Robot Operating System) Message Parser
"""

from threading import Thread
import struct
import math

import numpy as np

from osgar.bus import BusShutdownException
from osgar.lib.quaternion import euler_zyx

from subt import ign_pb2

MAX_TOPIC_NAME_LENGTH = 256


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


def packCmdVel(speed, angularSpeed):
    return struct.pack("dddddd", speed, 0, 0, 0, 0, angularSpeed)


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

    az, ay, ax = euler_zyx(orientation)  # quaternion, rotations along axis: yaw, pitch, roll

    return linearAcceleration, (az, ay, ax), orientation


def Xparse_image( data ):
    seq, stamp, stampNsec, frameIdLen = struct.unpack("IIII", data[:16])
#    print(seq, stamp, stampNsec, frameIdLen)
    data = data[16+frameIdLen:]

# from rosbag.py
def parse_raw_image(data, dump_filename=None):
    # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
    size = struct.unpack_from('<I', data)[0]
#    assert size == 230467, size  # expected size for raw image during experiment
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

    if encoding == b'32FC1':
        # depth is array of floats, OSGAR uses uint16 in millimeters
        # cut min & max (-inf and inf are used for clipping)
        arr = np.frombuffer(data[pos:pos + image_arr_size], dtype=np.dtype('f'))*1000
        arr = np.clip(arr, 1, 0xFFFF)
        arr = np.ndarray.astype(arr, dtype=np.dtype('H'))
    elif encoding == b'16UC1':
        # depth is array as uint16, similar to OSGAR
        arr = np.frombuffer(data[pos:pos + image_arr_size], dtype=np.dtype('H'))

    elif encoding == b'rgb8':
        # not compressed image data
        img = np.frombuffer(data[pos:pos+image_arr_size], dtype=np.dtype('B'))
        img = np.reshape(img, (height, width, 3))
        return img

    else:
        assert False, encoding  # unsuported encoding

    if dump_filename is not None:
        with open(dump_filename, 'wb') as f:
            # RGB color format (PPM - Portable PixMap)
#            f.write(b'P6\n%d %d\n255\n' % (width, height))
#            f.write(data[pos:pos+image_arr_size])
            # Grayscale float format (PGM - Portable GrayMap)
            f.write(b'P5\n%d %d\n255\n' % (width, height))
            f.write(bytes([min(255, x//100) for x in arr]))
    return np.array(arr).reshape((height, width))


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
    angle_range_deg = int(round(math.degrees(params[1] - params[0])))
    pos += 7*4
    ranges_size = struct.unpack_from('<I', data, pos)[0]
    # assert ranges_size == 720, ranges_size
    pos += 4
    scan = struct.unpack_from('<' + 'f' * ranges_size, data, pos)
    pos += 4 * ranges_size
    intensities_size = struct.unpack_from('<I', data, pos)[0]
    # assert intensities_size == 720, intensities_size
    intensities = struct.unpack_from('<' + 'f' * intensities_size, data, pos)

    # conversion to int and millimeters
    scan = [int(x*1000) if x < 65.0 else 0 for x in scan]

    if angle_range_deg == 270:
        # SubT Virtual World
        return scan

    if angle_range_deg == 149:
        # NASA moon
        size = len(scan)
        assert size == 100, size  # by single sample
        ret = [0]*40 + scan + [0]*40
        return ret

    # scan from map for MOBoS, Maria, K2, ...
    to_cut = int(len(scan) / 360 * 45)  # original scan is 360deg
    scan = scan[to_cut:len(scan)-to_cut]
    return scan


def parse_posestamped(data):
    size = struct.unpack_from('<I', data)[0]
    pos = 4
    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]
#    print(frame_id, timestamp_sec, timestamp_nsec)
    pos += frame_id_size
    # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html
    # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html

    x, y, z = struct.unpack_from('<ddd', data, pos)
    pos += 3 * 8
    ori = struct.unpack_from('<dddd', data, pos)
    pos += 4 * 8

    #q0, q1, q2, q3 = ori  # quaternion
    #ax =  math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
    #ay =  math.asin(2*(q0*q2-q3*q1))
    #az =  math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
    return (x, y, z), (ori)

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

    #q0, q1, q2, q3 = ori  # quaternion
    #ax =  math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
    #ay =  math.asin(2*(q0*q2-q3*q1))
    #az =  math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
    return timestamp_sec, (x, y), euler_zyx(ori)


def parse_points(data):
    # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
    size = struct.unpack_from('<I', data)[0]
#    assert size == 2457728, size  # expected size for cloud points (beware of variable header! i.e. this assert is wrong in general)
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
#     assert point_field_size == 4, point_field_size  # RGBD? - cloud == 5
#    print(data[pos:pos + 20])
    for i in range(point_field_size):
        str_size = struct.unpack_from('<I', data, pos)[0]
        pos += 4
        assert str_size < 10, str_size
        print(data[pos:pos + str_size])
        pos += str_size
        offset, datatype, count = struct.unpack_from('<IBI', data, pos)
        print(offset, datatype, count)
        assert datatype in [7, 4], datatype  # uint8 FLOAT32 = 7, uint8 UINT16=4
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


def parse_clock(data):
    size = struct.unpack_from('<I', data)[0]
    assert size == 8, size  # clock does not contain type, only NTP time
    pos = 4
    timestamp_sec, timestamp_nsec = struct.unpack_from('<II', data, pos)
    return (timestamp_sec, timestamp_nsec)


def parse_bool(data):
    size = struct.unpack_from('<I', data)[0]
    assert size == 1, size  # std_msgs::Bool does not contain type, only 1 byte of data
    val = data[4]
    assert val in [0, 1], val
    return val != 0


def parse_joint_state(data):
    # sensor_msgs/JointState.msg
    # Header
    # string[] name
    # float64[] position
    # float64[] velocity
    # float64[] effort
    size = struct.unpack_from('<I', data)[0]
    pos = 4
    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]
    pos += frame_id_size
    size = struct.unpack_from('<I', data, pos)[0]  # number of joints
    pos += 4
    name = []
    for i in range(size):
        str_len = struct.unpack_from('<I', data, pos)[0]
        pos += 4
        name.append(data[pos:pos+str_len])
        pos += str_len

    tmp = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    assert size == tmp, (size, tmp)  # number of joints
    position = struct.unpack_from('<' + 'd'*size, data, pos)
    pos += 8*size

    tmp = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    assert size == tmp, (size, tmp)  # number of joints
    velocity = struct.unpack_from('<' + 'd'*size, data, pos)
    pos += 8*size

    tmp = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    assert size == tmp, (size, tmp)  # number of joints
    effort = struct.unpack_from('<' + 'd'*size, data, pos)
    pos += 8*size

    return name, position, velocity, effort


def get_frame_id(data):
    size = struct.unpack_from('<I', data)[0]
    pos = 4
    if size == 8:
        # exception for /clock ?!
        return b'/clock'
    if size == 1:
        # exception for Bool -> /gas_detected ?!
        return b'/gas_detected'

    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]
    return frame_id


def parse_volatile(data):
    # NASA Space Robotics Challenge 2
    # __slots__ = ['header','vol_type','vol_index','shadowed_state','distance_to']
    # _slot_types = ['std_msgs/Header','string','int32','bool','float32']
    size = struct.unpack_from('<I', data)[0]
    pos = 4
    assert size + 4 == len(data), (size, len(data))  # it is going to be variable -> remove the assert
    seq, timestamp_sec, timestamp_nsec, frame_id_size = struct.unpack_from('<IIII', data, pos)
    pos += 4 + 4 + 4 + 4
    frame_id = data[pos:pos+frame_id_size]  # b'scout_1/volatile_sensor'
    pos += frame_id_size
    size = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    vol_type = data[pos:pos+size]  # b'methanol'
    pos += size
    vol_index, shadowed_state, distance_to = struct.unpack_from('<iBf', data, pos)
    return [vol_type.decode('ascii'), distance_to, vol_index]

def parse_bucket(data):
    # NASA Space Robotics Challenge 2
    # __slots__ = ['vol_type','vol_index','mass_in_bucket']
    # _slot_types = ['string','int32','float32']
    size = struct.unpack_from('<I', data)[0]
    pos = 4
    assert size + 4 == len(data), (size, len(data))  # it is going to be variable -> remove the assert
    vol_type_size = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    vol_type = data[pos:pos+vol_type_size]  # b'methanol'
    pos += vol_type_size
    vol_index, mass_in_bucket = struct.unpack_from('<if', data, pos)
    return [vol_type.decode('ascii'), vol_index, mass_in_bucket]

def parse_topic(topic_type, data):
    """parse general topic"""
    if topic_type == 'srcp2_msgs/Qual1ScoringMsg':
        assert len(data) == 8, (len(data), data)
        size = struct.unpack_from('<I', data)[0]
        pos = 4
        assert size == 4, size
        # __slots__ = ['score']
        # _slot_types = ['int32']
        return struct.unpack_from('<I', data, pos)
    elif topic_type == 'srcp2_msgs/HaulerMsg':
        assert len(data) == 134, (len(data), data)
        # vol_type: [ice, ethene, methane, carbon_mono, carbon_dio, ammonia, hydrogen_sul, sulfur_dio]
        # mass_per_type: [0.0, 0.0, 0.0, 0.0, 0.0, 70.26217651367188, 0.0, 0.0]
        vol_type_arr = []
        mass_arr = []
        size = struct.unpack_from('<I', data)[0]
        pos = 4
        for i in range(8):
            vol_type_len = struct.unpack_from('<I', data, pos)[0]
            pos += 4
            vol_type = data[pos:pos+vol_type_len]
            vol_type_arr.append(vol_type.decode('ascii'))
            pos += vol_type_len
        for i in range(8):
            mass = struct.unpack_from('<f', data, pos)[0]
            mass_arr.append(mass)
            pos += 4
        return [vol_type_arr, mass_arr]
    elif topic_type == 'srcp2_msgs/Qual2ScoringMsg':
        assert len(data) == 142, (len(data), data)
        # __slots__ = ['vol_type', 'points_per_type', 'num_of_dumps', 'total_score']
        # _slot_types = ['string[8]', 'int32[8]', 'int32', 'float32']
        # let's ignore names of volatile types
        record = struct.unpack_from('<IIIIIIIIIf', data, len(data) - 10*4)
        # print(record)
        return [sum(record[:8]), record[8]]  # score and attempts
    elif topic_type == 'srcp2_msgs/Qual3ScoringMsg':
        assert len(data) == 12, (len(data), data)
        size = struct.unpack_from('<I', data)[0]
        pos = 4
        assert size == 8, size
        # __slots__ = ['score','calls']
        # _slot_types = ['int32','int32']
        return list(struct.unpack_from('<II', data, pos))  # score and calls
    elif topic_type == 'std_msgs/String':
        size = struct.unpack_from('<I', data)[0]
        pos = 4
        msg_len = struct.unpack_from('<I', data, pos)[0]
        pos += 4
        string = data[pos:pos+msg_len]
        return string.decode('ascii')
    elif topic_type == 'srcp2_msgs/ExcavatorMsg':
        return parse_bucket(data)
    elif topic_type == 'sensor_msgs/JointState':
        return parse_joint_state(data)
    elif topic_type == 'srcp2_msgs/VolSensorMsg':
        return parse_volatile(data)
    elif topic_type == 'sensor_msgs/LaserScan':
        return parse_laser(data)
    elif topic_type == 'geometry_msgs/PoseStamped':
        return parse_posestamped(data)
    elif topic_type == 'sensor_msgs/Imu':
        return parse_imu(data)
    elif topic_type == 'std_msgs/Bool':
        return parse_bool(data)
    elif topic_type == 'sensor_msgs/CompressedImage':
        image = parse_jpeg_image(data)  # , dump_filename='nasa.jpg')
        return image
    else:
        assert False, topic_type


class ROSMsgParser(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)
        outputs = ["rot", "acc", "scan", "image", "pose2d", "sim_time_sec", "sim_clock", "cmd", "origin", "gas_detected",
                   "depth:null", "t265_rot", "orientation", "debug", "radio", "base_station",
                    "joint_name", "joint_position", "joint_velocity", "joint_effort"]
        self.topics = config.get('topics', [])
        for row in self.topics:
            topic_name, topic_type = row[:2]
            if len(row) > 2:  # extra rename
                outputs.append(row[2])
            else:
                outputs.append(topic_name)  # default use topic name
        bus.register(*outputs)

        self.bus = bus
        self._buf = b''

        self.topic_type = config.get('topic_type')
        self.timestamp_sec = None
        self.timestamp_nsec = None

        # initial message contains structure
        self.header = None
        self.count = 0
        self.downsample = config.get('downsample', 1)

        self.desired_speed = None  # m/s
        self.desired_angular_speed = None
        # alternative 3D speed description used in ROS Twist message
        self.desired_speed_3d, self.desired_angular_speed_3d = None, None
        self.gas_detected = None  # this SubT Virtual specific :-(

        self.joint_name = None  # unknown

    def publish_desired_speed(self):
        if self.desired_speed is not None:
            cmd = b'cmd_vel %f %f' % (self.desired_speed, self.desired_angular_speed)
        elif self.desired_speed_3d is not None:
            cmd = b'cmd_vel_3d %f %f %f %f %f %f' % tuple(self.desired_speed_3d + self.desired_angular_speed_3d)
        else:
            # desired speed is not defined - valid state for waiting drone - do not send anything!
            return
        self.bus.publish('cmd', cmd)

    def get_packet(self):
        data = self._buf
        if len(data) < 4:
            return None
        size = struct.unpack_from('<I', data, 0)[0]
        print(size, len(self._buf))
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
        if data.startswith(b'origin'):
            s = data.split()
            if b'ERROR' in s:
                self.bus.publish('origin', s[1:])
            else:
                self.bus.publish('origin', [s[1]] + [float(x) for x in s[2:]])
            return
        if data.startswith(b'depth'):
            depth = parse_raw_image(data[5:])
            self.bus.publish('depth', depth)
            return
        if data.startswith(b'radio '):
            s = data[6:].split(b' ')
            addr = s[0]
            msg = b' '.join(s[1:])
            if addr == b'base_station':
                artifact_score = ign_pb2.ArtifactScore()
                try:
                    artifact_score.ParseFromString(msg)
                except Exception as e:
                    print(e)
                    print(msg)
                    self.bus.report_error(exception=str(e), msg=msg)
                    return
                apos = [artifact_score.artifact.pose.position.x,
                        artifact_score.artifact.pose.position.y,
                        artifact_score.artifact.pose.position.z]
                new_msg = dict(
                    report_id=artifact_score.report_id,
                    artifact_type=artifact_score.artifact.type,
                    artifact_position=apos,
                    report_status=artifact_score.report_status,
                    score_change=artifact_score.score_change,
                )
                self.bus.publish('base_station', new_msg)
            else:
                self.bus.publish('radio', [addr, msg])
            return
        if data.startswith(b'points'):
            return
        if data.startswith(b'debug'):
            self.bus.publish('debug', data[6:])  # with space
            return
        frame_id = get_frame_id(data)
 #       print(frame_id)
        # TODO parse properly header "frame ID"
        #if frame_id.endswith(b'camera_depth_frame'):
        #    import pdb
        #    pdb.set_trace()
        if frame_id.endswith(b'/base_link/camera_front') or frame_id.endswith(b'/base_link/camera_down') or frame_id.endswith(b'camera_color_optical_frame'):
            # used to be self.topic_type == 'sensor_msgs/CompressedImage'
            self.bus.publish('image', parse_jpeg_image(packet))
        elif frame_id.endswith(b'base_link/front_laser') or frame_id.endswith(b'base_link/laser_front'):  # self.topic_type == 'sensor_msgs/LaserScan':
            self.count += 1
            if self.count % self.downsample != 0:
                return
            self.bus.publish('scan', parse_laser(packet))
        elif frame_id.endswith(b'odom'):  #self.topic_type == 'nav_msgs/Odometry':
            __, (x, y),rot = parse_odom(packet)
            self.bus.publish('pose2d', [round(x*1000),
                                        round(y*1000),
                                        round(math.degrees(rot[0])*100)])
            self.bus.publish('t265_rot', [round(math.degrees(angle)*100)
                                     for angle in rot])

            #workaround for not existing /clock on MOBoS
            self.publish_desired_speed()

        elif frame_id.endswith(b'/base_link/imu_sensor'):  # self.topic_type == 'std_msgs/Imu':
            acc, rot, orientation = parse_imu(packet)
            self.bus.publish('rot', [round(math.degrees(angle)*100)
                                     for angle in rot])
            self.bus.publish('acc', [round(x * 1000) for x in acc])
            self.bus.publish('orientation', list(orientation))
        elif frame_id.endswith(b'/clock'):
            prev = self.timestamp_sec
            self.timestamp_sec, self.timestamp_nsec = parse_clock(packet)
            self.bus.publish('sim_clock', [self.timestamp_sec, self.timestamp_nsec//1000])  # publish us resolution (~Python.timedelta)
            if prev != self.timestamp_sec:
                self.bus.publish('sim_time_sec', self.timestamp_sec)

            ms = self.timestamp_nsec//1000000
            if self.timestamp_sec > 0 and ms % 50 == 0:  # 20Hz
                self.publish_desired_speed()
        elif frame_id.endswith(b'/gas_detected'):
            # send only status change
            if self.gas_detected != parse_bool(packet):
                self.gas_detected = parse_bool(packet)
                self.bus.publish('gas_detected', self.gas_detected)

        elif b'\0' in packet[:MAX_TOPIC_NAME_LENGTH]:
            ros_name = packet[:packet.index(b'\0')].decode('ascii')
            for row in self.topics:
                n, t = row[:2]
                new_name = n if len(row) == 2 else row[2]

                if ros_name == n:
                    result = parse_topic(t, packet[len(ros_name) + 1:])
                    if t == 'sensor_msgs/JointState':
                        name, position, velocity, effort = result

                        # publish names only on change
                        if self.joint_name != name:
                            self.bus.publish('joint_name', list(name))
                            self.joint_name = name

                        self.bus.publish('joint_position', list(position))
                        self.bus.publish('joint_velocity', list(velocity))
                        self.bus.publish('joint_effort', list(effort))
                    elif t == 'sensor_msgs/Imu':
                        acc, rot, orientation = result
                        self.bus.publish('rot', [round(math.degrees(angle)*100)
                                                 for angle in rot])  # this is deprecated and will be removed
                        self.bus.publish('acc', [round(x * 1000) for x in acc])  # potential name conflict
                        self.bus.publish(new_name, list(orientation))  # typically 'orientation'
                    elif t == 'sensor_msgs/LaserScan':
                        self.count += 1
                        if self.count % self.downsample == 0:
                            self.bus.publish(new_name, list(result))  # typically 'scan'
                    else:
                        self.bus.publish(new_name, result)

    def slot_desired_speed(self, timestamp, data):
        self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)
        self.desired_speed_3d, self.desired_angular_speed_3d = None, None

    def slot_desired_speed_3d(self, timestamp, data):
        self.desired_speed, self.desired_angular_speed = None, None
        self.desired_speed_3d, self.desired_angular_speed_3d = data

    def slot_stdout(self, timestamp, data):
        cmd = b'stdout ' + bytes(data, 'utf-8')  # redirect to ROS_INFO
        self.bus.publish('cmd', cmd)

    def slot_request_origin(self, timestamp, data):
        cmd = b'request_origin'
        self.bus.publish('cmd', cmd)

    def slot_broadcast(self, timestamp, data):
        cmd = b'broadcast ' + data  # data should be already type bytes
        self.bus.publish('cmd', cmd)

    def run(self):
        try:
            while True:
                timestamp, channel, data = self.bus.listen()
                if channel == 'raw':
                    self.slot_raw(timestamp, data)
                elif channel == 'desired_speed':
                    self.slot_desired_speed(timestamp, data)
                elif channel == 'desired_speed_3d':
                    self.slot_desired_speed_3d(timestamp, data)
                elif channel == 'stdout':
                    self.slot_stdout(timestamp, data)
                elif channel == 'request_origin':
                    self.slot_request_origin(timestamp, data)
                elif channel == 'broadcast':
                    self.slot_broadcast(timestamp, data)
                else:
                    assert False, channel  # unsupported input channel
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
