Examples of OSGAR usage
=======================

This is place for some examples of robot and scenarios where OSGAR was used.

Create your own robot driver (myrobot)
--------------------------------------

This example demonstrates how to extend existing OSGAR to support also your
specific robot. Here only simple simulation is used so it should run without
need to have any extra hardware.


Async I/O with AsyncNode (asyncecho)
----------------------------------

The [AsyncEcho example](asyncecho/README.md) connects an asynchronous TCP server
to an application through the OSGAR bus. It demonstrates async message handlers,
event-loop startup, and connection cleanup using `osgar.asyncnode.AsyncNode`.
It runs locally without extra hardware or an external server.


SICK Robot Day 2018 (sick2018)
------------------------------

The German manufacture of LIDARs (laser scanner SICK) organizes every two years
competition in Waldkirch/Germany. The company provides testing samples of
sensors to participating universities. In 2018 it was TiM 571.

Robot Eduro was equipped with lidar, camera and hand with magnets to collect
red metal balls. This example contains code used in the 2nd run, where Eduro
scored 3 balls and finished on the 3rd place in the competition.

For more information see: 
       https://robotika.cz/competitions/sick-robot-day/2018/

