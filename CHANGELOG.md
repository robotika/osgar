# Changelog

[Full Changelog](https://github.com/robotika/osgar/compare/v0.3.0...master)

## [v0.3.0](https://github.com/robotika/osgar/tree/v0.3.0) (2020-11-30)
[Full Changelog](https://github.com/robotika/osgar/compare/v0.2.0...v0.3.0)

**osgar:**
- Base for SubT Tunnel/Urban/Cave Circuits and Moon project
- Raise minimum python version to 3.6
- ZMQRouter - run nodes in separate processes
- New Bus API
- Drivers for robots: Kloubak K2, K3, MOBoS, Maria
- New sensor drivers:
   - Velodyne driver (#626)
   - LoRa (Long Range Radio)
   - RealSense2 (RGBD and Tracking camera)
   - winsen_gas_detector
   - LORD IMU 3DM-GX5-25
   - e-stop for SubT
   - USB lidar
   - Peak CAN
   - vESC driver for experiments with motor controller
   - usb cameras on chained usb hubs
   - SICK lidar TIM310
- osgar.lib.config: allow overrides in dict merging (#612) 
- osgar.lib.unittest: add custom TestCase (#562) 
- Upgrade rosmsg to provide multiple devices (#545) 
- logzeromq 
   - provide Request/Response option for ROS services (#514)
   - add conditional log saving based on config['save_data']
- osgar.bus: track max delay during recording
- osgar.logger
   - add walltime to format string
   - make --times default behavior and --raw change to dump raw data
   - make --stat default behavior and use --all to dump all streams  
- osgar.replay: show modules available for replay
- serialize
   - add option to store packed data
   - serialize numpy arrays
- canserial.py - transition from raw bytes to PCAN triplets [addr, payload, flags]
- osgar.lib.quaternion support
- Lazyload drivers
- Implement ReplayDriver for reprocessing of logfiles
- Add pyusb into requirements
- Add opencv to requirements
- (over 400 master commits related to OSGAR)

**osgar-tools:**
- Upgrade lidarview
  - window resizable (#575)
  - display multiple fields in the title bar (#572)
  - add --bbox option to draw object bounding box (#464)
  - add options for lidar limit and window size
  - fix visual overflow in depth images
  - save image based on selected camera
  - draw K3 robot with two joints 
  - integrated 2nd lidar for Kloubak robots
  - add parameter --jump to given time
  - add parameter for optional --title
  - use numpy and colormap
  - allow keyframes stream for faster search
  - support lidar FOV controlled by command line --deg
  - add Framer (LogIndexedReader based) for faster replay
- log2video
  - create mp4 next to source logfile 
  - add --camera2 option for dual camera setup
  - add option for horizontal flip (upside down mounted camera)
  - add option --end-time-sec for video cut
- strip - added tool logfile size reduction

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

