Logging
=======

Motivation
----------

Logging is something very basic, used almost in all everyday programs.  The
term ,,logging'' is probably rather associated with textual logging of events
with different level of importance (debug, info, warning and error).  Such logs
have typically informative value but they are not sufficient in robot
development.  What we mean by "logging" is rather "data/event/communication
logging", i.e.  recording of all inputs used by the control system to generate
appropriate outputs.  Outputs are also logged (commands for the motors and
other effectors) and they are later used for verification purposes.

The typical OSGAR test scenario looks like this:
- test program is deployed on the robot
- robot performs the task
- logged data are moved to PC (with user friendly environment)
- test is re-run on PC to verify results repeatability

Note, that is some cases you can merge some of the steps, if for example the
robot is directly controlled from your laptop, or you work on robot PC remotely
and you consider it comfortable enough.

Probably the most important point is the last one.  The logged data have to be
complete in order that you can repeat the whole run with identical results.
Note, that this means that if you for example use probabilistic algorithms they
have to use fixed seed for random generator.  Multi-threaded systems have to
provide some way of deterministic scheduling.  The program should not contain
any Sleep() calls or dependencies on current system data/time.

.. \footnote{If date/time is required the return value has to be also separately logged}

Such limitations will probably many people consider cumbersome, but the benefit
is priceless.  We find it now very hard to work on any system not fulfilling
this basic assumption, and if there is no choice then our first step would be
to convert it (for example with additional log data) to this stage.

Why is it so big deal?  The robots work autonomously, typically much faster
then you can follow the run of the control program.  The program can work fine
for some period of time, but then once it fails.  This is something like
BlackBox used in airplanes, when you try to investigate the cause of the
crash.  This does not have to be that fatal in robotics, but the same way how
to increase the safety of flights via revision of BlackBoxes the same way you
can systematically increase the reliability of your robotic system.

