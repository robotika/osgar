import unittest
from unittest.mock import MagicMock
import math

from osgar.drivers.rosmsg import (ROSMsgParser, parse_jpeg_image, parse_laser,
                                  parse_odom, parse_imu, parse_points)


class ROSMsgParserTest(unittest.TestCase):

    def Xtest_parse_imu(self):
        r = ROSMsgParser(config={}, bus=None)
#        with open('imu_data.txt', 'rb') as f:
        with open('imu-x3.bin', 'rb') as f:
            r._buf += f.read()
            index = 0
            packet = r.get_packet()  # first packet is structure file
            while packet is not None:
                if index > 0:
                    ori = parse_imu(packet)
#                    q0, q1, q2, q3 = ori  # quaternion
                    
#                    x =  math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
#                    y =  math.asin(2*(q0*q2-q3*q1))
#                    z =  math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
#                    print([math.degrees(a) for a in [x, y, z]])

                packet = r.get_packet()
                index += 1
#                if index > 10000:
#                    break

    def Xtest_parse_image(self):
        r = ROSMsgParser(config={}, bus=None)
        with open('image_raw2.bin', 'rb') as f:
            r._buf += f.read()
            index = 0
            packet = r.get_packet()  # first packet is structure file
            while packet is not None:
                if index > 0:
                    parse_jpeg_image(packet, 'dump_%03d.jpg' % index)
                packet = r.get_packet()
                index += 1
                if index > 10:
                    break

    def Xtest_parse_laser(self):
        r = ROSMsgParser(config={}, bus=None)
        with open('laser_data2.bin', 'rb') as f:
            r._buf += f.read()
            index = 0
            packet = r.get_packet()  # first packet is structure file
            while packet is not None:
                if index > 0:
                    scan = parse_laser(packet)
                    print(scan)
                    self.assertEqual(len(scan), 720)
                    self.assertEqual(type(scan[0]), int)
                packet = r.get_packet()
                index += 1
                if index > 10:
                    break

    def Xtest_parse_odom(self):
        r = ROSMsgParser(config={}, bus=None)
        with open('odom_data.bin', 'rb') as f:
#        with open('x3-odom.bin', 'rb') as f:
            r._buf += f.read()
            index = 0
            packet = r.get_packet()  # first packet is structure file
            while packet is not None:
                if index > 0:
                    parse_odom(packet)
                packet = r.get_packet()
                index += 1
                if index > 10:
                    break

    def Xtest_parse_points(self):
        r = ROSMsgParser(config={}, bus=None)
        with open('point_data.bin', 'rb') as f:
            r._buf += f.read()
            index = 0
            packet = r.get_packet()  # first packet is structure file
            while packet is not None:
                if index > 0:
                    parse_points(packet)
                packet = r.get_packet()
                index += 1
                if index > 10:
                    break

    def test_zeromq_msg(self):
        data = bytes.fromhex(''.join("""4f010000f0e81100b45b00000054ea2a1700000058322f626173655f6c696e6
b2f696d755f73656e736f72a38f3ec2f5e0cebde6ce52f114ef483e0b68c7347249eabcfffffffff
fffef3f0000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000668fb2132
4db1bbf38d354b601e5eebef97da241c07601bf00000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000dda408b11c52a5bfe8f2195ae6107d3f36a73823e79b2340000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000""".split()))
        acc, angle = parse_imu(data)
        self.assertEqual(acc, (-0.041642090426032154, 0.007096195041978538, 9.804497814813299))
        self.assertEqual(angle, (-5.838168389364585e-15, -2.322151633890432e-08, -3.141592653477457))

# vim: expandtab sw=4 ts=4
