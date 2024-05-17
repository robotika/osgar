import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.drivers.vanjee import VanJeeLidar

class VanJeeLidarTest(unittest.TestCase):

    def test_usage(self):
        config = {}
        handler = MagicMock()
        lidar = VanJeeLidar(config, bus=handler)

    def test_id_overflow(self):
        packet = bytes.fromhex('ffaa056400000000781c0102190c000001041098b61504b11804b31104c81704b71504b21804b51104c91704b81504b31804b71104ca1704b91504b41804b71104cb1704ba1504b61804b71004cc1704ba1504b71804b81004ce1704ba1504b91804b91004cf1704bb1504ba1804bb1004d01704bc1504bb1704bd1004d11704bd1504bb1804bf1004d21704bf1504bc1804c01004d31704c01504bd1804c11104d41704c21504bf1804c21104d51704c41504c01804c11104d71704c51504c11804c11104d81704c61504c31804c11104da1704c81504c41804c31104db1704c91504c51804c51104dd1704c91504c61804c71104de1704ca1504c71704c91104df1704cb1504c91804cb1104e01704cc1504ca1804cc1104e11704cd1504ca1804cd1104e11704cf1504cb1804cd1104e21704d01504cc1804cd1104e21704d11504cd1804cd1104e11704d21504cf1804ce1104e21704d21504d01804cf1104e21704d31504d21804d11004e31704d51504d41804d21004e51704d71504d61804d40f04e71704d91504d81804d60e04e91604db1504d91804d70e04ec1704dd1504da1804d80d04ed1704df1504db1704da0e04ef1704e01504dc1804dc0f04f11704e01504dd1804de0f04f21704e11504de1704e01004f41704e21504df1804e21004f71704e31504e11704e41004f91704e51504e31804e51004fc1704e71504e51804e71004fe1704e91504e71704e81005001704eb1504e81704e91005011704ed1504e91704ea1005031704ee1504eb0000001005051704f01504ec1704ed1005061604f11504ed1704ef1005081704f21504ef1704f11005091604f31504f11704f21005091604f51504f21704f410050a1704f71504f41804f610050a1604f81504f61804f810050a1604fa1504f71704fb0e00001604fc1504f81704fd0b00001604fd1504fa1705000000001604ff1504fb1705020800001605011504fe18050400000016050415050118050608037a1605061505031805070703401605091505061805090702f316050c15050817050b00000016050e15050918050d0602e916051115050a18050f0702e616051315050c1705110702e416051515050e1705140802e41605171505101805160802e51605191505131805190802e716051b15051518051b00000016051d15051718051d07000016051f15051918051f0a000016052115051a1805210c040516052416051d1805230d041a1605261605201805260d042e1605281505221705280c000016052b15052417052b00000016052d15052718052e0d00001605301505291705310c000016053315052c1805330c04ec16053515052f1705360c04e81605391505311705390c04f316053b15053417053b0d04f816053d15053717053e0d04c81605401505391705410d04c216054215053b1705440d050f16054415053d1705470d051416054715054017054a0d051616054a15054317054c0d051816054d15054517054f0d051a1605501505471705520d051a15055315054a1705560d000016055615054c1805590e000016055915054e17055c0d000016055b1505511705600d051716055d1505531705630d05161605601505571705650d051516056215055a1705680d051216056515055e17056a00000016056815056117056d0e000016056b15056417056f00000016056e1505681705720f000016057115056d17057500000016057515057100000000000000000015057517057c11000016057b15057818057f0b000016057e15057b1705810b000016058215057e1705830a00001605851505801705850000001605881505821705880b000016058c15058617058c00000016058f15058917059000000016059315058d1705940000001505961505911705980000000000000000000002eeee')
        config = {}
        handler = MagicMock()
        lidar = VanJeeLidar(config, bus=handler)
        lidar.last_frame = 0xFFFF
        lidar.on_raw(packet)  # should not raise AssertionError: (65535, 0)

# vim: expandtab sw=4 ts=4
