"""
  Moon Scout Driver
"""

# source: (limited access)
#   https://gitlab.com/scheducation/srcp2-competitors/-/wikis/Documentation/API/Simulation_API


# SCOUT
# Volatile Sensor
# /scout_n/volatile_sensor  srcp2_msgs/VolSensorMsg


import math
from osgar.lib.mathex import normalizeAnglePIPI

from osgar.node import Node
from moon.rover import Rover

class Scout(Rover):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('object_reached')
        self.last_volatile_distance = None
        self.last_vol_index = None

    def on_volatile(self, data):
        
        # called by incoming volatile sensor report (among other sources)
        # 0 vol_type, 1 distance_to, 2 vol_index
        artifact_type = data[0]  # meters ... TODO distinguish CubeSat, volatiles, ProcessingPlant
        if artifact_type in ['ice', 'ethene', 'methane', 'methanol', 'carbon_dio', 'ammonia', 'hydrogen_sul', 'sulfur_dio']:
            distance_to = data[1]
            vol_index = data[2]

            if self.last_volatile_distance is None:
                self.last_volatile_distance = distance_to
            elif self.last_volatile_distance > distance_to:
    #            print ("Volatile detection %d, getting closer: %f" % (vol_index, distance_to))
                self.last_volatile_distance = distance_to
            elif self.last_vol_index is None or vol_index != self.last_vol_index:
                self.last_vol_index = vol_index
                self.last_volatile_distance = None
                # TODO: this must be adjusted to report the position of the sensor, not the robot (which NASA will update their code for at some point)
                # the best known distance was in reference to mutual position of the sensor and the volatile
                print (self.time, "Volatile detection, starting to go further, reporting")

                self.bus.publish('object_reached', artifact_type)
            else:
                self.last_volatile_distance = None
    #            print ("Previously visited volatile %d, not reporting" % vol_index)
            

# vim: expandtab sw=4 ts=4
