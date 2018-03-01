from .gps import GPS
from .imu import IMU
from .spider import Spider
from .logserial import LogSerial

# dictionary of all available drivers
all_drivers = dict(gps=GPS, imu=IMU, spider=Spider, serial=LogSerial)

