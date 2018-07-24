from .gps import GPS
from .imu import IMU
from .spider import Spider
from .logserial import LogSerial
from .canserial import CANSerial
from .simulator import SpiderSimulator
from .logsocket import LogTCP
from .sicklidar import SICKLidar
from .johndeere import JohnDeere

# dictionary of all available drivers
all_drivers = dict(gps=GPS, imu=IMU, spider=Spider, serial=LogSerial,
                   can=CANSerial, simulator=SpiderSimulator,
                   tcp=LogTCP, lidar=SICKLidar,
                   johndeere=JohnDeere)

