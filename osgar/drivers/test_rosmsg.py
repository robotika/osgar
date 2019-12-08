import unittest

from osgar.drivers.rosmsg import (ROSMsgParser, parse_jpeg_image, parse_laser,
                                  parse_odom, parse_imu, parse_points, get_frame_id,
                                  parse_bool)


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

    def test_get_frame_id(self):
        data = b'\x08\x00\x00\x00\x02\x00\x00\x00\x00\x06\x81\x14'
        self.assertEqual(get_frame_id(data), b'/clock')
        data = b'\x01\x00\x00\x00\x00'
        self.assertEqual(get_frame_id(data), b'/gas_detected')

    def test_parse_bool(self):
        data = b'\x01\x00\x00\x00\x00'
        self.assertEqual(parse_bool(data), False)

# vim: expandtab sw=4 ts=4
