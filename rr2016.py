#!/usr/bin/python
"""
  Navigation along a straight road in Pisek (contest Robotem Rovne)
  usage:
       ./rr2016.py <task> [<metalog> [<F>]]
"""
import sys
from can import CAN, DummyMemoryLog
from johndeer import JohnDeer


def ver0():
    # no other sensors
    com = DummyMemoryLog()
    com.data += [0x80>>3, 0]*10000  # just test/hack without CAN module
    robot = JohnDeer(can=CAN(com=com))
    robot.desired_speed = 0.5
    start_time = robot.time
    while robot.time - start_time < 10.0:
        robot.update()
    robot.stop()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    ver0()

# vim: expandtab sw=4 ts=4 

