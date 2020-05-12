import unittest

from unittest.mock import MagicMock

from osgar.bus import Bus
from subt.main import SubTChallenge
from subt.trace import distance3D

from subt import simulation


import sys
if '-v' not in sys.argv and '--verbose' not in sys.argv:
    simulation.verbose(False)
    def print(*args):
        pass


def entrance_reached(sim):
    corrected = [(rr - oo) for rr, oo in zip(sim.xyz, sim.origin)]
    goal = [2.5, 0, 0]
    if distance3D(corrected, goal) < 2:
        return True
    return False


class SubTChallengeTest(unittest.TestCase):

    def test_maybe_remember_artifact(self):
        config = {'max_speed': 0.5, 'walldist': 0.9, 'timeout': 600, 'symmetric': True,
                  'right_wall': True}
        bus = MagicMock()
        game = SubTChallenge(config, bus)

        artf_data = ['TYPE_BACKPACK', -1614, 1886]
        artf_xyz = (0, 0, 0)
        self.assertTrue(game.maybe_remember_artifact(artf_data, artf_xyz))

        # 2nd report should be ignored
        self.assertEqual(game.maybe_remember_artifact(artf_data, artf_xyz), False)

    def test_go_to_entrance(self):
        config = {'virtual_world': True, 'max_speed': 1.0, 'walldist': 0.8, 'timeout': 600, 'symmetric': False, 'right_wall': 'auto'}
        log = simulation.SimLogger()
        bus = Bus(log)
        app = SubTChallenge(config, bus.handle('app'))
        sim = simulation.Simulation(bus.handle('sim'), end_condition=entrance_reached)
        print("connecting:")
        for o in bus.handle('sim').out:
            print(f'  sim.{o} → app.{o}')
            bus.connect(f'sim.{o}', f'app.{o}')
        for o in bus.handle('app').out:
            print(f'  app.{o} → sim.{o}')
            bus.connect(f'app.{o}', f'sim.{o}')
        print("done.")
        app.start()
        sim.start()
        sim.join()
        app.request_stop()
        app.join()
        self.assertTrue(entrance_reached(sim))



# vim: expandtab sw=4 ts=4

