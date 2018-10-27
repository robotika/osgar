import unittest
from column import *

class ColumnTest(unittest.TestCase):

    def test_polar2coord(self):
        # test55
        self.assertEqual(polar2coord( (46, 2.808) ), (1.9506007042488642, -2.019906159350932) )
        self.assertEqual(polar2coord( (196, 1.194) ), (-1.1477464649503528, 0.32911100284549705) )

        self.assertEqual(polar2coord( (89, 1.762) ), (0.030751140142492875, -1.7617316388655615) )
        self.assertEqual(polar2coord( (269, 1.21) ), (-0.021117411789113007, 1.2098157111392334) )

    def test_col_dist(self):
        self.assertAlmostEqual(col_dist( (89, 1.762), (269, 1.21) ), 2.972)
        self.assertAlmostEqual(col_dist( (46, 2.808), (196, 1.194) ), 3.888, 3)  # hmmm, probably bug??

    def test_analyse_pose(self):
        p = analyse_pose(prev_pose=None, new_data=[(89, 1.762), (269, 1.21)])
        self.assertEqual(p[0], (2.6194324487249787e-16, 0.2760000000000001, -0.017453292519943098))
        
        # select best matching pair
        p = analyse_pose(None, [(46, 2.8080000000000003), (169, 5.9500000000000002), (196, 1.194)])
        self.assertEqual(p[0], (0.43115108250453843, 0.8306320134455586, -0.922098513045361))


if __name__ == "__main__":
    unittest.main() 
  
# vim: expandtab sw=4 ts=4 

