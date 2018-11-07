OSGAR
=====

Open Source Garden/Generic Autonomous Robot (Python library)

OSGAR is lightweight multi-platform library targeting record and replay of
multiple `nodes` (modules = sensors, robots, applications) logged into single file. It
has similar goals as ROS or ADTF, but is minimalistic. It should run on
different operating systems and also low end devices like Raspberry PI Zero. 

![John Deere X300R](http://robotika.cz/competitions/roboorienteering/2016/jd-nav2.jpg)

References at
http://robotika.cz/robots/osgar/

Video: https://youtu.be/KiDnPsnLmLU

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

