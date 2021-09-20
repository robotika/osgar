import unittest
from unittest.mock import patch, MagicMock

# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys
sys.modules['cbor'] = MagicMock()

import numpy as np

from subt.mapping_server import create_map


class MappingServerTest(unittest.TestCase):

    def test_empty_map(self):
        m = create_map(np.zeros(shape=(1, 0, 3), dtype=np.float32))
        self.assertEqual(m['data'], b'')


# vim: expandtab sw=4 ts=4
