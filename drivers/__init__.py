from .gps import GPS
from .imu import IMU
from .spider import Spider

# dictionary of all available drivers
all_drivers = dict(gps=GPS, imu=IMU, spider=Spider)

