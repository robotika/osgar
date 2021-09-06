import unittest
from unittest.mock import patch

from subt.mapping_server import create_empty_map


class MappingServerTest(unittest.TestCase):

    def test_empty_map(self):
        m = create_empty_map()
        self.assertEqual(m['data'], b'')


# vim: expandtab sw=4 ts=4
