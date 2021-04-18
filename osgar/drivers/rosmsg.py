"""
  ROS (Robot Operating System) Message Parser
"""

from datetime import timedelta
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
}

def packCmdVel(speed, angularSpeed):
    return struct.pack("dddddd", speed, 0, 0, 0, 0, angularSpeed)


def parse_clock(data):
    size = struct.unpack_from('<I', data)[0]
    assert size == 8, size  # clock does not contain type, only NTP time
    pos = 4
    timestamp_sec, timestamp_nsec = struct.unpack_from('<II', data, pos)
    return (timestamp_sec, timestamp_nsec)


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
    else:
        assert False, topic_type


class ROSMsgParser(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)
        outputs = ["sim_time_sec", "sim_clock", "cmd",
                   "t265_rot", "debug", "radio", "base_station",
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

        self.last_cmd_timestamp = None

        # initial message contains structure
        self.header = None
        self.count = 0
        self.downsample = config.get('downsample', 1)

        self.desired_speed = None  # m/s
        self.desired_angular_speed = None
        # alternative 3D speed description used in ROS Twist message
        self.desired_speed_3d, self.desired_angular_speed_3d = None, None

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
        if frame_id.endswith(b'/clock'):
            prev = self.timestamp_sec
            self.timestamp_sec, self.timestamp_nsec = parse_clock(packet)
            self.bus.publish('sim_clock', [self.timestamp_sec, self.timestamp_nsec//1000])  # publish us resolution (~Python.timedelta)
            if prev != self.timestamp_sec:
                self.bus.publish('sim_time_sec', self.timestamp_sec)

            now = timedelta(seconds=self.timestamp_sec,
                            milliseconds=self.timestamp_nsec//1000000)
            if self.timestamp_sec > 0 and (
                    self.last_cmd_timestamp is None or
                    now - self.last_cmd_timestamp >= timedelta(milliseconds=48)):  # ~20 Hz
                self.publish_desired_speed()
                self.last_cmd_timestamp = now

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
                elif channel == 'broadcast':
                    self.slot_broadcast(timestamp, data)
                else:
                    assert False, channel  # unsupported input channel
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
