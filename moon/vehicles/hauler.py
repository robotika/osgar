"""
  Moon Hauler Driver
"""

# source: (limited access)
#   https://gitlab.com/scheducation/srcp2-competitors/-/wikis/Documentation/API/Simulation_API

# Hauler Bin
#  /hauler_n/bin_joint_controller/command

# Info
#  /hauler_n/bin_info


import math
from osgar.lib.mathex import normalizeAnglePIPI

from osgar.node import Node
from moon.rover import Rover

class Hauler(Rover):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        
# vim: expandtab sw=4 ts=4
