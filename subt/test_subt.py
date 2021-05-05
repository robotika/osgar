import unittest
import logging
import math

from unittest.mock import MagicMock, call

from osgar.bus import Bus
from osgar.lib import quaternion
from subt.main import SubTChallenge
from subt.trace import distance3D

from subt import simulation

g_logger = logging.getLogger(__name__)


def entrance_reached(sim):
    corrected = [(rr - oo) for rr, oo in zip(sim.xyz, sim.origin)]
    goal = [0.5, 0, 0]  # note, that in real run the Y coordinate depends on choise left/righ
    if distance3D(corrected, goal) < 2:
        return True
    return False


class SubTChallengeTest(unittest.TestCase):

    def test_go_to_entrance(self):
        config = {'virtual_world': True, 'max_speed': 1.0, 'walldist': 0.8, 'timeout': 600, 'symmetric': False, 'right_wall': 'auto'}
        bus = Bus(simulation.SimLogger())
        app = SubTChallenge(config, bus.handle('app'))
        sim = simulation.Simulation(bus.handle('sim'), end_condition=entrance_reached)
        g_logger.info("connecting:")
        for o in bus.handle('sim').out:
            if o == 'pose2d':
                continue  # connect 'pose3d' only
            g_logger.info(f'  sim.{o} → app.{o}')
            bus.connect(f'sim.{o}', f'app.{o}')
        for o in bus.handle('app').out:
            g_logger.info(f'  app.{o} → sim.{o}')
            bus.connect(f'app.{o}', f'sim.{o}')
        g_logger.info("done.")
        app.start()
        sim.start()
        sim.join()
        app.request_stop()
        app.join()
        self.assertTrue(entrance_reached(sim))


if __name__ == "__main__":
    import sys
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    unittest.main()

# vim: expandtab sw=4 ts=4

