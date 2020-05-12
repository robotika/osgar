import unittest
from unittest.mock import MagicMock

from subt.main import SubTChallenge


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


# vim: expandtab sw=4 ts=4

