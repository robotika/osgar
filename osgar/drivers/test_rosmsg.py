import unittest
from unittest.mock import MagicMock

from osgar.drivers.rosmsg import (ROSMsgParser, parse_jpeg_image, parse_laser,
                                  parse_odom, parse_imu, parse_points, get_frame_id,
                                  parse_bool, parse_volatile, parse_bucket, parse_topic, parse_posestamped)


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

    def test_parse_volatile(self):
        data = b"<\x00\x00\x00\x8c\x00\x00\x009\x00\x00\x00\x00'\xb9)\x17\x00\x00\x00" + \
               b"scout_1/volatile_sensor\x08\x00\x00\x00methanol\n\x00\x00\x00\x01\xfb\xde`?"
        self.assertEqual(parse_volatile(data), ['methanol', 0.8784024119377136, 10])

    def test_parse_bucket(self):
        data = b'\x16\x00\x00\x00\n\x00\x00\x00sulfur_dio\x1b\x00\x00\x00\xd3\xa7\xabA'
        self.assertEqual(parse_bucket(data), ['sulfur_dio', 27, 21.456945419311523])

    def test_parse_posestamped(self):
        data = b"K\x00\x00\x00\x16\x01\x00\x00s\x06\x00\x00@O\xa86\x03\x00\x00\x00map\xcb\x80M+\xc9\xcf\xce?\xb8\xb5\x9c\x04\xfb\x98\xbb\xbfp\xbb\x0e\xe8'U\xd1\xbfa`9&\x83F\x84\xbf\xce\xf7O\x9f\x90e\xbe\xbf\xf7,N\xba\xd7\xcf\xb0?\x8b\xc3D\xf5\xd4\xb3\xef?"
        self.assertEqual(parse_posestamped(data), ((0.24071611990348737, -0.10780304777718797, -0.2708225027262676), (-0.00990011654282358, -0.11873725785696296, 0.06567142771216426, 0.9907021322634234)))

    def test_parse_score_qual2(self):
        data = b'\x87\x00\x00\x00\x03\x00\x00\x00ice\x06\x00\x00\x00ethene\x07\x00\x00\x00methane' + \
               b'\x08\x00\x00\x00carbon_mono\n\x00\x00\x00carbon_dio\x07\x00\x00\x00ammonia\x0c\x00\x00\x00' + \
               b'hydrogen_sul\n\x00\x00\x00sulfur_dio\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00' + \
               b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00' + \
               b'\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        self.assertEqual(parse_topic('srcp2_msgs/Qual2ScoringMsg', data), [0, 0])

    def test_imu_math_domain_error(self):
        data = b'J\x01\x00\x00\x0c\x18\x01\x00\x05\t\x00\x00\x80\xc0\r\x16\x12\x00\x00\x00scout' + \
               b'_1///imu_linkl\x18B2\xb2e\x95?\xb8T\x9b\x8f\xcc\x1c\xe5\xbf\xb5dw\x91\xf7h\xd8?' + \
               b'\xe5\xaaF\x00\xf1\xb9\xe8\xbf|\x14\xaeG\xe1zd?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00|\x14\xaeG\xe1zd?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00|\x14\xaeG\xe1zd?\x80\xa6\xa6\xeam\x1b~?x_\xd7\x19\xb6\x02\xc5?\t\xd4\xe3*\xbbx\xb6\xbf|\x14\xaeG\xe1zd?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00|\x14\xaeG\xe1zd?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00|\x14\xaeG\xe1zd?\xb8\xec\x82\xe3\xe1[\xd1\xbf\x14K\xb9TQ]\x03\xc0\xdb\xfc\x99\xf1\xb8\x05\x00@|\x14\xaeG\xe1zd?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00|\x14\xaeG\xe1zd?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00|\x14\xaeG\xe1zd?'
        parse_imu(data)  # it failed to compute asin from -1.003659325094236

    def test_radio(self):
        data = b'radio X30F60R [0, 100, 30]\n'
        bus = MagicMock()
        r = ROSMsgParser(config={}, bus=bus)
        r.slot_raw(timestamp=None, data=data)
        bus.publish.assert_called_with('radio', [b'X30F60R', b'[0, 100, 30]\n'])

# vim: expandtab sw=4 ts=4
