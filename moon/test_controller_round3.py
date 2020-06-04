import unittest
from random import Random
import math

import numpy as np
from shapely.geometry import LineString
from shapely.geometry import Point

from moon.controller_round3 import best_fit_circle

def pol2cart(rho, phi):
    x = rho * math.cos(phi)
    y = rho * math.sin(phi)
    return(x, y)

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def plot_test_case(circle, lines):
    circle_center_x, circle_center_y, circle_radius = circle
    x_l, y_l = lines


    fig, ax = plt.subplots()
    ax.set_aspect(1)
    ax.add_artist(plt.Circle((circle_center_x, circle_center_y), circle_radius))

    for i in range(len(lines[0])):
        plt.plot([0,x_l[i]],[0,y_l[i]])
    plt.axis([-50, 50, -50, 50])
    plt.show()

has_matplotlib = False

class Round3CircleTest(unittest.TestCase):
        
    def test_round3_circle(self):
        # generate random points on 2.6rad arc with random radius between 5 and 20
        # view is at (0,0)
        # circle is at random point and has random radius
        rand = Random(0)
        for _ in range(10):
            circle_distance = rand.randint(10,20)
            circle_angle = rand.random() * 2 * math.pi
            radius = rand.randint(2,6)
            center_x, center_y = pol2cart(circle_distance, circle_angle)
            circle_angle = circle_angle + (1 - rand.random() * 2) * math.pi / 4

            p = Point(center_x,center_y)
            c = p.buffer(radius).boundary

            x_l=[]
            y_l=[]
            min_index = rand.randint(0,20)
            max_index = rand.randint(80,100)
            for inc in range(min_index, max_index):
                angle = circle_angle - 1.3 + inc * 0.0262626260519
                ox, oy = pol2cart(2*circle_distance, angle)
                l = LineString([(0,0), (ox, oy)])
                i = c.intersection(l)
                if i.is_empty:
                    continue

                i_x = i.geoms[0].coords[0][0]
                i_y = i.geoms[0].coords[0][1]
                # TODO: pick the nearest intersections
                dst,ang = cart2pol(i_x, i_y)
                # add noise to distance by 5%
                dst = dst * (1 + rand.random()/10 - 0.05)
                i_x2, i_y2 = pol2cart(dst, ang)
                x_l.append(i_x2)
                y_l.append(i_y2)

            if has_matplotlib:
                plot_test_case((center_x, center_y, radius),(x_l, y_l))
            (cx, cy, cr) = best_fit_circle(x_l, y_l)
            np.testing.assert_almost_equal((center_x/200, center_y/200, radius/200), (cx/200, cy/200, cr/200),2)

if __name__ == '__main__':
    # you may need to define your environment variable to point to the main osgar folder
    # eg export PYTHONPATH=/osgar
    try:
        from matplotlib import pyplot as plt
        has_matplotlib = True
    except ImportError:
        has_matplotlib = False
    unittest.main()
