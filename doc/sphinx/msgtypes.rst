Message Types
=============

.. glossary::

   emergency_stop
      Boolean if Emergency STOP was pressed. Should be reported on system start
      and on change.

   encoders
      List of differences of all available encoders since the last update. For two
      wheeled robot they shell be ordered [left, right]. Positive difference
      corresponds to forward motion. The scaling factor is specific to robot
      and can change over time (preasure in tires, payload). The main purpose
      of this message type is an advanced position estimation.

   gps_position
      [longitude_ms, latitude_ms] - GPS coordinates in WGS84 system.

   pose2d
      [x, y, heading] - position in meters with heading in radians
      (anticlockwise).

   desired_speed
      [speed, angular_speed] - command used for differentially driven robots.
      Speed is in meters per second and angular_speed in radians per second.



Note about scaling
------------------

Integer values are preferred to float numbers, so fractional numbers are used.
The scaling has to be performed manually at the moment but future
implementation should hide it. Float meters are scaled by factor 1000 and
rounded, i.e. sent as millimeters. Angles in radians are scaled by 5729
(exactly math.degrees(1)*100), i.e. it corresponds to 1/100th of degree).
The same holds for speed and angular speed.

