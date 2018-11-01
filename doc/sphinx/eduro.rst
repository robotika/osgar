Robot Eduro
===========

Eduro was designed and created by Jan Roubicek back in 2007 as EDUcational
RObot for the university. The electronics was manufactured by Tomas Roubicek,
and it includes CAN network modules using CANopen protocol, custom made stepper
motor drivers with encoders SMAC (Stepper Motor - Adaptive Control) and power management.

.. image:: https://robotika.cz/competitions/fieldrobot/2010/eduro-prices2.jpg

The original "small" Eduro was rebuilt into outdoor version "Eduro Maxi" in
2010. Since then it participated in many outdoor competitions and gained
several prices including 1st place in 
`Field Robot Event <https://robotika.cz/competitions/fieldrobot/>`_ and 
`Robotour <https://robotour.cz>`_ contests.

The robot is differentially driven with two stepper motors. It is tricycle with very
high maneuverability. The maximal speed is limited to 1 meter per second.

The communication uses SYNC [#f1]_ signal, after which all CANopen modules transmit
their status. The default is 20Hz cycle, but can be modified. The encoder
status is sent independently for left and right wheel. Once the motor driver is
in operational mode it requires update every 200ms otherwise internal
watchdog will, due to safety reasons (original Eduro was capable of speed
4m/s), motor disable and switch to pre-operation.

The motor controller tries to achieve desired speed immediately within cycle
period, so it is recommended to use "ramping" and split commands into several
steps. The acceleration (ramp slope) depends on robot payload and surface. If
you set it high you probably loose traction with the ground, which also means
that you loose otherwise very precise odometry information.

The robot uses `APU2 board <https://www.pcengines.ch/apu2.htm>`_ with serial COM
for CAN bridge, 2x USB ports and 3x Ethernet. There is also Wi-Fi module
available. This system enables extension to various sensors: lidars, GPS,
IP-cameras, etc.

For more info see: http://eduro.cz/ or https://robotika.cz/robots/eduro/

.. [#f1] There is only SYS module compiled with CANopen SYNC_producer stack. In
  theory any module including PC can be configured to generate SYNC.

