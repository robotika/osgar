Drivers
=======

The "drivers" directory contains implementation of all drivers for various
devices. Sensors, effectors as well as the mobile platforms are included.
The driver takes care of a single device, or better to say a single
communication stream (serial.Serial for example). The data are collected,
logged and parsed for further processing.

Logging
-------

There is at least one stream logged per driver - the input for sensors and
the output for effectors. If bidirectional communication is required then
both streams are logged. The "stream ID" is part of the driver configuration.

Use cases
---------

There are two basic use cases: real run and replay. In real run real port
is opened and dedicated Thread::run() takes care of reading data with timeout
and the processing.

In the "replay" mode the run is single-threaded and the processing is
simulated from logged data. This mode is mainly designed for debugging,
development of parsers and also verification of identical outputs on
different platforms (robot PC vs. notebook for example).


