import unittest
import os.path
from subt.dummy_darpa_server import GameLogic, dist3d


class DummyDarpaServerTest(unittest.TestCase):

    def test_game_logic(self):
        filename = os.path.join(os.path.dirname(__file__), 'config', 'darpa-artf.csv')
        # content
        """
artf, x, y, z
Backpack, 2, -21.2, 0.5
Cell Phone, 5, 5, 5
        """
        game = GameLogic(filename)
        self.assertEqual(game.score, 0)

        self.assertTrue(game.report_artf('Cell Phone', (5, 5, 5)))  # exact match
        self.assertEqual(game.score, 1)

        self.assertFalse(game.report_artf('Cell Phone', (5, 5, 5)))  # already reported

    def test_dist3d(self):
        self.assertAlmostEqual(dist3d((0, 0, 0), (1, 0, 0)), 1.0)
        self.assertAlmostEqual(dist3d((1, 2, 3), (1+3, 2+4, 3)), 5.0)


# vim: expandtab sw=4 ts=4

