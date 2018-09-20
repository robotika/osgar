OSGAR
=====

Open Source Garden/Generic Autonomous Robot (Python library)

![John Deere X300R](http://robotika.cz/competitions/roboorienteering/2016/jd-nav2.jpg)

References at
http://robotika.cz/robots/osgar/

Video: https://youtu.be/KiDnPsnLmLU

# Notes/Howto

There has been serious development to extend original code suited mainly for
modified John Deere X300R garden tractor to more general tool. Now it runs
on more than four machines, including autonomous boat. It provides logging
and replay facility, so simple logging of multiple inputs is unified and
stored in binary format with timestamps.

The current code is under `osgar` directory and the remaining code is ported
now. If you are using OSGAR directly from sources please make sure that
you setup PYTHONPATH to the root of this project.

The example configurations are stored in `config` folder. JSON files are
used.

## Examples

### Collect data from sensor(s)

There is a `osgar/robot.py` to run data collection based on given configuration file.
If you would like to collect GPS data available on serial port use modified version
of `test-windows-gps.json` for Windows or `test-gps-imu.json` for Linux.


```
python ./osgar/robot.py run config/test-windows-gps.json
```

To replay existing log use:
```
python ./osgar/replay.py --module <module name> <log file name>
```

