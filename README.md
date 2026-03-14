OSGAR
=====

**Open Source Garden/Generic Autonomous Robot (Python library)**

OSGAR is a lightweight, multi-platform library for recording and replaying data from multiple `nodes` (modules such as sensors, robots, and applications) logged into a single file. It is designed to be minimalistic, modular, and robust, capable of running on everything from high-end workstations to low-end devices like the Raspberry Pi Zero.

OSGAR was originally developed by [robotika.cz](https://robotika.cz) and has been successfully deployed in several international robotics challenges.

# Key Features

*   **Robust Logging:** All port data is logged with microsecond-resolution timestamps into a single `.log` file.
*   **Deterministic Replay:** Replaying recorded data produces identical outputs, enabling reliable debugging of complex sensor interactions and algorithms in a controlled environment.
*   **Modular Architecture:** Robots are built as a set of independent modules connected via a communication "bus," defined in a simple JSON configuration.
*   **Minimalistic & Multi-platform:** Written in Python, with minimal dependencies, ensuring it runs on various operating systems and hardware.

# Applications & Robots

OSGAR has been the core software framework for a variety of robotic platforms and prestigious competitions. See [OSGAR Platforms](osgar/platforms/README.md) for more details.

<img src="http://robotika.cz/competitions/roboorienteering/2016/jd-nav2.jpg" alt="John Deere X300R" width="600">

### DARPA Subterranean Challenge (SubT)
Team Robotika used OSGAR to coordinate a heterogeneous fleet of robots (wheeled, tracked, and flying) to map and search complex underground environments.

<img src="https://robotika.cz/competitions/subtchallenge/tunnel-circuit/day3-0742-kloubak.jpg" alt="DARPA SubT" width="600">

### DARPA Triage Challenge (DTC)
OSGAR powers a fleet of **Matty** robots (M01-M05) equipped with non-contact sensors to detect physiological signs in casualties during mass-casualty incidents.

<img src="https://robotika.cz/competitions/dtc/phase2/matty-all-hw-completed.jpg" alt="Matty robots for DTC" width="600">

### Field Robot Event
OSGAR-based robots, including the "Matty twins," have successfully competed in the Field Robot Event, an international competition for autonomous robots in agriculture.

<img src="https://robotika.cz/competitions/fieldrobot/2025/matty-twins-prizes.jpg" alt="Field Robot Event" width="600">

# Documentation

For more detailed information, please refer to:
*   [Technical Guide (English)](https://robotika.github.io/osgar/index.html)
*   [Czech Guide (Průvodce OSGARem)](https://robotika.cz/guide/osgar/cs)

# Architecture

Robot is set of modules. Every module has input and output ports, described
together with the connections in the config file. All ports are logged with
timestamp (microsecond resolution). The module is typically an instance of a
`driver`, whose init arguments are also in config file.


# Notes regarding GitHub repository

The current development code is under `osgar` directory.
If you are using OSGAR directly from sources please make sure that
you setup PYTHONPATH to the root of this project.

The example configurations are stored in `config` folder. JSON files are
used.

## Examples

### Collect data from sensor(s)

There is a [osgar/record.py](https://github.com/robotika/osgar/blob/master/osgar/record.py)
to run data collection based on given configuration file.
If you would like to collect GPS data available on serial port use modified version
of [test-windows-gps.json](https://github.com/robotika/osgar/blob/master/config/test-windows-gps.json)
for Windows or 
[test-gps-imu.json](https://github.com/robotika/osgar/blob/master/config/test-gps-imu.json) for Linux.


```
python -m osgar.record config/test-windows-gps.json
```

To replay existing log use:
```
python -m osgar.replay --module <module name> <log file name>
```
