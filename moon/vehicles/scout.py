"""
  Moon Scout Driver
"""

# source: (limited access)
#   https://gitlab.com/scheducation/srcp2-competitors/-/wikis/Documentation/API/Simulation_API


# SCOUT
# Volatile Sensor
# /scout_n/volatile_sensor  srcp2_msgs/VolSensorMsg
# sensor delay: rosparam get /volatile_detection_service_delay_range


import math
from datetime import timedelta

from osgar.lib.mathex import normalizeAnglePIPI

from osgar.node import Node
from moon.vehicles.rover import Rover


class Scout(Rover):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('object_reached')
        self.last_volatile_distance = None
        self.last_vol_index = None
        self.last_vol_timestamp = None

    def on_volatile(self, data):

        # called by incoming volatile sensor report (among other sources)
        # 0 vol_type, 1 distance_to, 2 vol_index
        artifact_type = data[0]  # meters ... TODO distinguish CubeSat, volatiles, ProcessingPlant
        if artifact_type in ['ice', 'ethene', 'methane', 'carbon_mono', 'carbon_dio', 'ammonia', 'hydrogen_sul', 'sulfur_dio']:
            distance_to = data[1]
            vol_index = data[2]

            if self.last_volatile_distance is None:
                self.last_volatile_distance = distance_to
            elif self.last_volatile_distance > distance_to:
    #            print ("Volatile detection %d, getting closer: %f" % (vol_index, distance_to))
                self.last_volatile_distance = distance_to
            elif self.last_vol_index is None or self.sim_time - self.last_vol_timestamp > timedelta(seconds=30):
                # if no previous volatile or a volatile was last reported more than 30 seconds ago
                self.last_vol_index = vol_index
                self.last_vol_timestamp = self.sim_time
                self.last_volatile_distance = None
                # TODO: this must be adjusted to report the position of the sensor, not the robot (which NASA will update their code for at some point)
                # the best known distance was in reference to mutual position of the sensor and the volatile
                print (self.sim_time, "Volatile detection, starting to go further, reporting")

                self.bus.publish('object_reached', [artifact_type, vol_index])
            else:
                self.last_volatile_distance = None
    #            print ("Previously visited volatile %d, not reporting" % vol_index)


# vim: expandtab sw=4 ts=4
