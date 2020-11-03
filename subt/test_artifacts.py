import unittest
from unittest.mock import MagicMock, call
from pathlib import Path

import cv2

from subt.artifacts import artf_in_scan, ArtifactDetector

curdir = Path(__file__).parent

class ArtifactDetectorTest(unittest.TestCase):

    def test_count_red(self):
        #img = cv2.imread(str(curdir/'test_data/artf-backpack.jpg'))
        #self.assertEqual(count_red(img), (1576, 47, 87, 272, 319))

        #img = cv2.imread(str(curdir/'test_data/artf-extinguisher.jpg'))
        #self.assertEqual(count_red(img), (527, 19, 56, 0, 19))

        img = cv2.imread(str(curdir/'test_data/artf-extinguisher-hd.jpg'))
        #self.assertEqual(count_red(img), (8221, 84, 221, 23, 107))

        img = cv2.imread(str(curdir/'test_data/artf-valve-hd.jpg'))
        #self.assertEqual(count_red(img), (187, 21, 30, 322, 343))

        img = cv2.imread(str(curdir/'test_data/artf-valve2-hd.jpg'))
        #self.assertEqual(count_red(img), (116, 1, 34, 344, 345))

    def test_artf_in_scan(self):
        scan = [
2305, 2292, 2273, 2260, 2238, 2235, 2204, 2205, 2187, 2136, 2163, 2144, 2129,
2122, 2116, 2087, 2082, 2057, 2071, 2065, 2042, 2037, 2022, 2011, 1832, 1814,
1807, 1796, 1778, 1787, 1773, 1774, 1767, 1754, 1747, 1723, 1708, 1728, 1716,
1701, 1723, 1730, 1731, 1778, 1777, 1826, 1830, 1820, 1832, 1799, 1816, 1811,
1797, 1781, 1782, 1782, 1784, 1776, 1754, 1761, 1749, 1739, 1752, 1764, 1748,
1735, 1725, 1711, 1720, 1706, 1722, 1690, 1704, 1710, 1691, 1697, 1688, 1690,
1681, 1684, 1680, 1679, 1673, 1671, 1660, 1654, 1662, 1654, 1649, 1645, 1658,
1655, 1627, 1665, 1636, 1642, 1652, 1647, 1622, 1633, 1630, 1641, 1627, 1642,
1621, 1614, 1620, 1621, 1619, 1628, 1623, 1633, 1614, 1623, 1614, 1624, 1614,
1634, 1615, 1602, 1614, 1632, 1618, 1625, 1610, 1612, 1611, 1618, 1639, 1615,
1605, 1643, 1620, 1629, 1635, 1622, 1564, 1506, 1480, 1489, 1497, 1476, 1476,
1500, 1496, 1488, 1499, 1494, 1495, 1520, 1493, 1505, 1494, 1514, 1509, 1503,
1512, 1522, 1524, 1535, 1684, 1696, 1697, 1694, 1693, 1695, 1683, 1700, 1707,
1706, 1709, 1711, 1716, 1717, 1714, 1729, 1725, 1745, 1758, 1737, 1744, 1756,
1756, 1764, 1777, 1781, 1794, 1798, 1805, 1808, 1824, 1807, 1825, 1828, 1825,
1839, 1840, 1870, 1873, 1863, 1865, 1866, 1881, 1881, 1892, 1897, 1913, 1923,
1917, 1959, 1936, 1949, 1955, 1969, 1975, 1998, 1995, 2026, 2013, 2036, 2030,
2042, 2074, 2072, 2109, 2095, 2120, 2116, 2122, 2132, 2149, 2117, 2107, 2097,
2090, 2061, 2060, 2061, 2046, 2073, 2078, 2088, 2119, 2124, 2143, 2131, 2155,
2179, 2189, 2428, 2461, 2480, 2472, 2506, 2541, 2536, 2557, 2605, 2624, 2631,
2647, 2681, 2695, 2742, 2756, 2763, 2818, 2822, 2844, 2871, 2908, 2929, 2976,
2989, 3023, 3048, 3083, 3109, 3106, 3085, 3061, 3071, 3066, 3072, 3074, 3103,
3147, 3189, 3214, 3581, 3651, 3690, 3751, 3810, 3864, 3924, 3977, 4036, 4085,
4162, 4228, 4230, 4196, 4192, 4195, 4188, 4260, 4293, 4852, 4937, 5025, 5143,
5236, 5326, 5371, 5408, 5387, 5376, 5413, 5527, 6238, 6396, 6563, 6619, 6588,
6584, 6657, 7549, 7769, 7809, 7812, 7801, 8865, 9057, 9050, 9031, 10296, 10285,
10276, 11525, 11506, 12764, 12761, 13992, 14000, 15233, 16474, 17722, 18973, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 19262, 19296, 18075, 18074, 16942,
16840, 16659, 15600, 15647, 15395, 14438, 14450, 14316, 13991, 13201, 13228,
13115, 12849, 12091, 12013, 12024, 11864, 11631, 10992, 10838, 10841, 10841,
8269, 8247, 8245, 8315, 9681, 9689, 9691, 9554, 9423, 9292, 9150, 8675, 8561,
8544, 8560, 8569, 8440, 8337, 8235, 8146, 8026, 7962, 7557, 7465, 7439, 7463,
7476, 7434, 7360, 7261, 7203, 7098, 7050, 6983, 6941,6863, 6568, 6475, 6421,
6378, 6393, 6467, 6458, 6387, 6339, 6272, 6211, 6179, 6133, 6079, 6037, 5989,
5926, 5906, 5840, 5823, 5532, 5508, 5468, 5430, 5432, 5457, 5492, 5486, 5454,
5430, 5378, 5364, 5320, 5287, 5241, 5228, 5208, 5185, 5159,5117, 5078, 5065,
5025, 4997, 4987, 4971, 4911, 4713, 4683, 4647, 4647, 4629, 4578, 4624, 4661,
4694, 4690, 4680, 4668, 4640, 4599, 4590, 4579, 4565, 4520, 4520, 4487, 4479,
4469, 4454, 4426, 4411, 4424, 4375, 4382, 4367, 4360, 4342, 4308,4316, 4295,
4269, 4248, 4271, 4074, 4053, 4055, 4040, 4020, 4011, 4000, 4016, 4025, 4078,
4130, 4133, 4136, 4091, 4081, 4093, 4082, 4057, 4042, 4046, 4048, 4037, 4032,
4037, 4020, 4001, 4032, 3995, 3994, 3992, 3993, 3997, 3961, 3971, 3960,3958,
3932, 3939, 3947, 3939, 3920, 3937, 3931, 3900, 3934, 3936, 3895, 3905, 3748,
3732, 3743, 3738, 3726, 3745, 3754, 3716, 3746, 3876, 3886, 3879, 3890, 3881,
3871, 3881, 3891, 3901, 3894, 3931, 3881, 3889, 3885, 3902, 3887, 3879,
3900, 3884, 3886, 3898, 3902, 3912, 3901, 3922, 3914, 3908, 3921, 3931, 3917,
3927, 3951, 3946, 3952, 3963, 3967, 3961, 3971, 3982, 3958, 3865, 3833, 3837,
3858, 3867, 3855, 3869, 3867, 3877, 4068, 4036, 4069, 4077, 4086, 4096, 4088,
4110, 4136, 4131, 4153, 4138, 4154, 4167, 4170, 4177, 4196, 4202, 4220,
4246, 4239, 4255, 4285, 4291, 4296, 4309, 4320, 4336, 4347, 4369, 4394,
4400, 4427, 4392, 4331, 4281, 4302, 4333, 4339, 4352, 4368, 4396, 4587,
4606, 4620, 4649, 4660, 4676, 4712, 4717, 4745, 4779, 4787, 4818, 4817,
4864, 4902, 4911, 4940, 4974, 4972, 5018, 5035, 5087, 5096, 5129, 5116,
5067, 5050, 5041, 5071, 5099, 5123, 5165, 5385, 5433, 5479]

        # artf_TYPE_VALVE_228.jpg based on red pixels
        self.assertEqual(artf_in_scan(scan, 1280, 247, 259), (2212, 8245))

        # the proper bounds based on image and the column
        self.assertEqual(artf_in_scan(scan, 1280, 152, 219), (2212, 8245))

    def test_electrical_box(self):
        img = cv2.imread(str(curdir/'test_data/artf-electrical-box.jpg'))
        #self.assertEqual(count_white(img), (865248, 1266, 959, 0, 1266))

    def test_missing_scan(self):
        deg_100th, dist_mm = artf_in_scan(scan=None, width=1280, img_x_min=100, img_x_max=200)
        self.assertEqual((deg_100th, dist_mm), (0, 0))

    def test_gas_detected(self):
        config = {
            'virtual_world': True
        }
        bus = MagicMock()
        detector = ArtifactDetector(config, bus)
        data = True

        detector.handle_gas_artifact(data)
        bus.publish.assert_called()

# vim: expandtab sw=4 ts=4

