from .gps import GPS
from .imu import IMU
from .spider import Spider
from .logserial import LogSerial
from .canserial import CANSerial
from .simulator import SpiderSimulator
from .logsocket import (LogTCPStaticIP, LogTCPDynamicIP, LogTCPServer,
                        LogUDP, LogHTTP)
from .sicklidar import SICKLidar
from .eduro import Eduro
from .rosproxy import ROSProxy
from .rosmsg import ROSMsgParser

# dictionary of all available drivers
all_drivers = dict(gps=GPS, imu=IMU, spider=Spider, serial=LogSerial,
                   can=CANSerial, simulator=SpiderSimulator,
                   tcp=LogTCPStaticIP, tcpdynamic=LogTCPDynamicIP,
                   tcpserver=LogTCPServer, udp=LogUDP, http=LogHTTP,
                   lidar=SICKLidar,
                   eduro=Eduro,
                   rosproxy=ROSProxy,
                   rosmsg=ROSMsgParser,
                   )

