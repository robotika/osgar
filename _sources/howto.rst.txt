How to control my robot with OSGAR?
===================================

Is OSGAR useful for me? How can I use it with my robot? That could be one of
the first questions when you encounter OSGAR Python library ...

- PC, Python3, Raspberry, pip install osgar


The first observation is that, similar to humans, every robot is specific, but
there are some common features. If you are lucky and have some commonly used
machine and sensors, all you need to do is pick appropriate configuration at
`osgar/config <https://github.com/robotika/osgar/tree/master/config>`_
and that's it, you are ready. But typically this is not the case. So
let's start with some hypothetical example, that you have differentially driven
robot controlled over serial line. You have also GPS connected via USB.

Attaching GPS
-------------

We will start with GPS, which is more common and you can easily buy one for
less than 20 USD (see `u-blox M8 <https://www.u-blox.com/en/product/neo-m8-series>`_ for example).
The GPS typically uses USB-serial converter and is
available on some COM port on Windows and /dev/tty* on Linux. We can use
already configuration available on
`github <https://github.com/robotika/osgar/blob/master/config/test-windows-gps.json>`_

.. code-block:: json

  {
    "version": 2,
    "robot": {
      "modules": {
        "gps": {
            "driver": "gps",
            "in": ["raw"],
            "out": ["position"],
            "init": {}
        },
        "gps_serial": {
            "driver": "serial",
            "in": [],
            "out": ["raw"],
            "init": {"port": "COM5", "speed": 4800}
        }
      },
      "links": [["gps_serial.raw", "gps.raw"]]    
    }
  }


You can see that there are two modules/nodes named "gps_serial" and "gps".
"gps_serial" directly talks with your GPS device over serial line and receives
bytes via "serial" driver. The "gps" is already NMEA parser of messages
received via "raw" named input. The output is then "position" - two elements
array with longitude and latitude in milliseconds (integer with scale factor
1/3600000).

OK, but how it fits into my robot?! Check the "gps_serial" "init" section and
setup your port ("COM5") and communication speed (4800). Save the configuration
(say you name it "myrobot.json") and now we can test it without any programming
that it works for your setup:

.. code ::

  python -m osgar.record myrobot.json --duration=10 --note "my first GPS test"

You can obtain all parameters for recorder via "-h" option:

.. code ::

  python -m osgar.record -h

In particular the configuration file parameter is required, but *duration* and
*note* are optional. Nevertheless I would highly recommend to add notes to your
robot trials as they are easy to add before start and priceless once you have
to analyze some older log files.

If everything went fine you will not see much yet: there should be newly
created logfile named *myrobot-YYMMDD_hhmmss.json* where *YYMMDD_hhmmss*
corresponds to date and time on your computer.

Let's look in your log file for collected data:

.. code ::

  python -m osgar.logger myrobot-YYMMDD_hhmmss.json --stream gps.position --times

Depending on your specific GPS you will see 10 lines with timestamp (time zero
corresponds to the beginning of the file) and your current position. If you
want raw NMEA data (to upload to `Robotour Marathon 2019 <https://robotika.cz/competitions/robotour/marathon-2019>`_ for example) use:

.. code ::

  python -m osgar.logger myrobot-YYMMDD_hhmmss.json --stream gps_serial.raw --raw > marathon.nmea

Is it working? If yes great, if not let us know or check the
`OSGAR issues <https://github.com/robotika/osgar/issues>`_.


Attaching motors
----------------

Now the harder part which will involve some programming, but you will also
learn how to use OSGAR as a library. Suppose that there is no driver for your
robot. In this case you will need to lookup the motor controller documentation
and if you are the author following then section could help you to reasonably define
the communication protocol.

Again let's assume you can control the motors via serial line. This time surely
on different port and maybe also with different communication speed but the
rest will look very similar to GPS setup. At the moment we do not have "named
driver" for your motor controller, so we will use (some would say misuse)
generic "app" for "application code" and talk to motors directly.

.. code-block:: json

  {
    "version": 2,
    "robot": {
      "modules": {
        "app": {
            "driver": "application",
            "in": ["raw"],
            "out": ["cmd"],
            "init": {"move_time_sec":4.0}
        },
        "motor_serial": {
            "driver": "serial",
            "in": ["raw"],
            "out": ["raw"],
            "init": {"port": "COM7", "speed": 38400}
        }
      },
      "links": [["motor_serial.raw", "app.raw"],
                ["app.cmd", "motor_serial.raw"]]
    }
  }

Note other slight differences in this configuration (we dropped GPS part for the
moment and we will integrate/merge it later). In particular communication
with *motor_serial* is bidirectional, there is input and output (both named
*raw* but the naming is up to you although there are some recommendations [TODO]).
Also the speed and port are different. There is also extra parameter
"move_time_sec" which will be later accessible within your code.

Now it is time to code *myrobot.py*:

.. code-block:: python

  class MyRobot:

      def __init__(self, config, bus):
          self.bus = bus
          self.move_time_sec = config['move_time_sec']

      def run(self):
          self.bus.publish('cmd', b'go\n')
          self.bus.sleep(self.move_time_sec)
          self.bus.publish('cmd', b'stop\n')
          self.bus.sleep(0.5)

This is minimalistic code which will send command "go", wait number of seconds
described in your configuration file, then "stop" and wait 0.5 second to see
slowing down in your log file. Yes, surely your robot does not know commands
like "go", maybe you need to send some PWM values like "127, 127\\n" for
maximal speed forward or your motor driver requires binary encoding and you
need to `struct.pack("BB", 127, 127)` to get your motors moving. That is the
very specific part to your robot.

The common part is that you can directly run this code with

.. code ::

  python -m osgar.record myconf.json --app myrobot.py --duration 5

XXX limit by apptime? OR any finishes?

After your test, when robot hopefully moved (BTW you already coded version for
`Robot go straight! <https://robotika.cz/competitions/robotem-rovne>`_ competition) you can again have a look at your logfile:

.. code ::

  python -m osgar.logger --list myconf-YYMMDD_hhmmss.log

will show available streems and

.. code ::

  python -m osgar.logger myconf-YYMMDD_hhmmss.log --stream app.cmd --times

will display commands your "application" sent to motors.

Note, that now you can also "replay" your log from real run:

.. code ::

  python -m osgar.replay myconf-YYMMDD_hhmmss.log --module app --app myrobot.py

XXX how to define "external app"?

Now, depending on your motor driver, it will pass without any error or with
warning that when you published your "stop" command the input queue was not
processed for a very long time (in our example 4s). This is something what we
will fix in the next step. Suppose that your motor controller reports status of
your encoders every second (typically with much faster cycle). So there are
pending messages your application did not read. Let's fix it:

.. code-block:: python

  from datetime import timedelta

  class MyRobot:

      def __init__(self, config, bus):
          self.bus = bus
          self.move_time_sec = config['move_time_sec']
          self.time = None

      def run(self):
          self.time = self.bus.publish('cmd', b'go\n')
          start_time = self.time
          while self.time - start_time < timedelta(seconds=self.move_time_sec):
              self.update()
          self.bus.publish('cmd', b'stop\n')

      def update(self):
          timestamp, channel, data = self.bus.listen()
          self.time = timestamp
          # TODO some processing with received data
          assert channel == 'raw', channel
          print(data)


OK, now the main difference is usage internal `update()` function instead of
`bus.sleep()`. It waits for input data (i.e. if your motor controller does not
send any data it will not work), updates "system time" and at the moment only
prints received data and verifies that you received data only from "raw" input.

What is nice on this development cycle is that you can now process your
collected data from real run and debug (or now develop) your parser for example.


Creation of driver
------------------

The application written specific for your motor controller is nice, but all you
achieved so far is that your robot moved almost straight for a couple of
seconds. What next? With your hardware setup, you can use GPS to navigate to
any GPS destination, and this algorithm is "generic" in the sense that other
types of robots may reuse it. The price you have to pay is to write a "driver"
with expected interfaces and then plug it in bigger setup.

TODO continue

