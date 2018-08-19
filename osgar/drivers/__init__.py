from .gps import GPS
from .imu import IMU
from .spider import Spider
from .logserial import LogSerial
from .canserial import CANSerial
from .simulator import SpiderSimulator
from .logsocket import LogTCP, LogUDP
from .sicklidar import SICKLidar

# dictionary of all available drivers
all_drivers = dict(gps=GPS, imu=IMU, spider=Spider, serial=LogSerial,
                   can=CANSerial, simulator=SpiderSimulator,
                   tcp=LogTCP, udp=LogUDP, lidar=SICKLidar)

