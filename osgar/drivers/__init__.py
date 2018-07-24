from .gps import GPS
from .imu import IMU
from .spider import Spider
from .logserial import LogSerial
from .canserial import CANSerial
from .simulator import SpiderSimulator
from .logsocket import LogTCP, LogUDP, LogHTTP
from .sicklidar import SICKLidar
from .eduro import Eduro
from .johndeere import JohnDeere

# dictionary of all available drivers
all_drivers = dict(gps=GPS, imu=IMU, spider=Spider, serial=LogSerial,
                   can=CANSerial, simulator=SpiderSimulator,
                   tcp=LogTCP, udp=LogUDP, http=LogHTTP,
                   lidar=SICKLidar,
                   eduro=Eduro,
                   johndeere=JohnDeere,
                   )

