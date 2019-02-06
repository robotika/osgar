# Changelog

[Full Changelog](https://github.com/robotika/osgar/compare/v0.2.0...master)

## [v0.2.0](https://github.com/robotika/osgar/tree/v0.2.0) (2019-02-07)
[Full Changelog](https://github.com/robotika/osgar/compare/v0.1.0...v0.2.0)

**osgar:**
- Use environment variable OSGAR_LOGS to define log output
- Provide base class `Node`
- Introduce application launcher
- Use LogReader directly as iterator
- Add explore node with follow wall function
- logger.py
  - optimization for lookup_stream_names
  - handle timestamp overflow
- Add OpenCV driver for camera logging
- Add driver for Cortexpilot (robot Robik)
- Extend TCP driver for "server" and "dynamic" option
- Support SICK LIDAR TiM551

**osgar-tools:**
- Add simple LIDAR viewer
  - draw accumulated points
  - add callback option for scan debugging
- log2video.py - add stream ID as optional parameter


## [v0.1.0](https://github.com/robotika/osgar/tree/v0.1.0) (2018-11-21)
[Full Changelog](https://github.com/robotika/osgar/compare/v0.0.1...v0.1.0)

**osgar:**
- Rename `robot.py` to `record.py` to better match record/replay pair
- Extend `bus` to log errors, sleep and return timestamp also for `publish()`
- Assert response time is not larger than 0.1s (i.e no long queue, sufficiently fast processing)
- Add support for robot Eduro
- Bugfix `sicklidar`
- Improve support for CANopen and restart modules
- Extend built-in drivers by general Python class

**osgar-tools:**
- Add `log2video` for conversion of logfiles into AVI files
- Add `log2pcap` for conversion of Velodyne logs for VeloView

**other:**
- Start generated documentation at http://robotika.github.io/osgar/
- Add `examples` directory with the first example `sick2018`
- Cleanup repository and move unsupported code to `_deprecated`


## [v0.0.1](https://github.com/robotika/osgar/tree/v0.0.1) (2018-10-21)

- Provide basic logging, communication and replay functionality
- Support protocols (I/O drivers): serial, CAN, UDP, TCP, HTTP
- Support sensors: GPS (NMEA), dGPS(uBlox 8), SICK TiM 571, IMU (NMEA $VNYMR)
- Support robots: Spider3 (with simple simulator)

