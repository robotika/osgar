#!/usr/bin/python
"""
  Hello World - minimalistic example
  usage:
       ./helloworld.py [-h] {run,replay} ...

positional arguments:
  {run,replay}  sub-command help
    run         run on real HW
    replay      replay from logfile

optional arguments:
  -h, --help       show this help message and exit
"""

from launcher import parse_and_launch
from driver import go_straight

with parse_and_launch() as (robot, __, __, __):
    go_straight(robot, distance=1.0, speed=0.3)

# vim: expandtab sw=4 ts=4 

