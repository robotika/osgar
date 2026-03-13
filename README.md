OSGAR
=====

**Open Source Garden/Generic Autonomous Robot (Python library)**

OSGAR is a lightweight, multi-platform library for recording and replaying data from multiple `nodes` (modules such as sensors, robots, and applications) logged into a single file. It is designed to be minimalistic, modular, and robust, capable of running on everything from high-end workstations to low-end devices like the Raspberry Pi Zero.

OSGAR was originally developed by [robotika.cz](https://robotika.cz) and has been successfully deployed in several international robotics challenges.

![John Deere X300R](http://robotika.cz/competitions/roboorienteering/2016/jd-nav2.jpg)

# Key Features

*   **Robust Logging:** All port data is logged with microsecond-resolution timestamps into a single `.log` file.
*   **Deterministic Replay:** Replaying recorded data produces identical outputs, enabling reliable debugging of complex sensor interactions and algorithms in a controlled environment.
*   **Modular Architecture:** Robots are built as a set of independent modules connected via a communication "bus," defined in a simple JSON configuration.
*   **Minimalistic & Multi-platform:** Written in Python, with minimal dependencies, ensuring it runs on various operating systems and hardware.

# Applications & Robots

OSGAR has been the core software framework for a variety of robotic platforms and prestigious competitions:

### DARPA Subterranean Challenge (SubT)
Team Robotika used OSGAR to coordinate a heterogeneous fleet of robots (wheeled, tracked, and flying) to map and search complex underground environments.
![DARPA SubT](https://robotika.cz/competitions/darpa-subt/darpa-subt.jpg)

### DARPA Triage Challenge (DTC)
OSGAR powers robots equipped with non-contact sensors to detect physiological signs in casualties during mass-casualty incidents.
![DARPA Triage Challenge](https://robotika.cz/competitions/darpa-triage/darpa-triage.jpg)

### Pat & Matty
**Pat** (Yuhesen FR-07 platform) and **Matty** (versatile terrain robots M01-M05) are used for testing advanced vision sensors (like OAK-D) and competing in outdoor challenges like Robotour.
![Pat](https://robotika.cz/robots/pat-a-mat/pat.jpg) ![Matty](https://robotika.cz/robots/pat-a-mat/matty.jpg)

# Documentation

For more detailed information, please refer to:
*   [Technical Guide (English)](https://osgar.readthedocs.io/)
*   [Czech Guide (Průvodce OSGARem)](https://robotika.cz/guide/osgar/cs)
*   [Reference at robotika.cz](http://robotika.cz/robots/osgar/)

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

