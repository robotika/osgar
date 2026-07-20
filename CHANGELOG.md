# Changelog

[Full Changelog](https://github.com/robotika/osgar/compare/v1.1.0...master)

## [v1.1.0](https://github.com/robotika/osgar/tree/v1.1.0) (2026-07-17)
[Full Changelog](https://github.com/robotika/osgar/compare/v1.0.0...v1.1.0)

**osgar:**
- New drivers & platform support:
  - Added Ouster LiDAR SDK driver (#1050)
  - Added simple DJI Tello mini-drone driver (#1061)
  - Added scan5 stream support to VanJee Lidar driver (#1068)
  - Added bumper support for Spider platform (#1054) and updated Spider configuration (#1053)
  - Support both Robotourist v0 and v1 formats (#1048)
- Luxonis OAK Camera (DepthAI v3) driver enhancements:
  - Added support for neural network grayscale input (#1072)
  - Added support for superblobs (#1062)
  - Added support for IMU orientation output (#1057)
  - Added parameter for auto exposure compensation (#1060)
  - Added additional driver configuration parameters (#1049)
- Core & API improvements:
  - Added support for config-driven channel overrides `:gz` and `:null` (#1069)
  - Added package-level `__version__` attribute to `osgar` (#1073)
  - Moved `self.verbose` attribute to common `Node` (#1058)
  - Consolidated `EmergencyStopException` into `osgar.exceptions` (#1052)
  - Added `LogReaderEx` utility (#1051)
  - Added option to restrict full laser/LiDAR scans to a narrow cone (#1065)
  - Added `--config` parameter to `osgar.logger` (#1047)
- Replay improvements:
  - Fixed initial timestamps in `osgar.replay` (#1070)
  - Extended replay functionality with `--params` option (#1063)
  - Improved `osgar.replay --output` to match the original recording (#1059)
  - Extended `osgar.replay --draw` with custom parameters and added Matty joint angle drawing (#1055)
  - Allowed interrupting forced replay on Ctrl+C (#1066)
- Deprecations & general updates:
  - Removed `subt` from setup packages (#1078)
  - Deprecated SubT (#1074)
  - Updated GitHub deployment workflows (#1076)
  - Configured sphinx-build to separate doctrees cache from output HTML (#1067)
  - Added "Deep Dive for OSGAR Developers" documentation (#1046)

**osgar-tools:**
- Added support for multiple neural network models in LidarView (#1071)
- Assert video creation with non-compatible image resolution in LidarView (#1075)

## [v1.0.0](https://github.com/robotika/osgar/tree/v1.0.0) (2026-03-14)
[Full Changelog](https://github.com/robotika/osgar/compare/v0.3.0...v1.0.0)

**osgar:**
- Base for SubT Finals (#894) and FRE2025
- Enforced `Node.on_MSG` callbacks
- Drivers for robots: Yuhesen FR-07, Matty, Skiddy, Deedee
- New sensor drivers:
   - Luxonis OAK cameras (added DepthAI v3 support, stereo images, visual odometry)
   - Support multiple neural networks at once on OAK cameras
   - VanJee lidar
   - Pozyx
   - system monitor (CPU, temperature, RAM)
   - RealSense D455 and L515
   - New Audio driver
- Support ZeroMQ PUB/SUB endpoints
- Support environment variables in configuration files
- Improved program termination on STOP with `osgar.terminator`
- Flush log file properly on Ctrl+C
- Provide `joint_angle` for Matty platform (compatible with Kloubak)
- Integrate Matty protocol version 8 (IMU data)
- Add color LED functionality to Matty platform
- Add tool for listing inputs and outputs of OSGAR modules
- (over 600 master commits since v0.3.0)

**osgar-tools:**
- Upgrade lidarview
  - use black color for undefined depth pixels
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
  - add option to mix video with audio
  - create mp4 next to source logfile 
  - add --camera2 option for dual camera setup
  - add option for horizontal flip (upside down mounted camera)
  - add option --end-time-sec for video cut
- replay: optionally ignore selected stream for output assert
- strip - added tool logfile size reduction

**other:**
- Improved documentation (intro, key features, application examples)
- Added MyRobot tutorial
- Provide `uv` environment support (`uv.lock`)
- Automated Sphinx documentation deployment

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
