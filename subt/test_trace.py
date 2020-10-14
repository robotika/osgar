
import unittest

from subt.trace import Trace

class Test(unittest.TestCase):

    def test_trace(self):
        st = Trace()
        self.assertEqual(len(st.trace), 0)  # unknown initial location

        st.update_trace((0, 0, 0))
        self.assertEqual(len(st.trace), 1)  # initialized with (0, 0, 0)

        st.update_trace((0, 0, 0))
        self.assertEqual(len(st.trace), 1)  # duplicities removed

        st.update_trace((1.0, 0, 0))
        self.assertEqual(len(st.trace), 2)

        # now move in 3D up
        st.update_trace((1.0, 0, 1.0))
        self.assertEqual(len(st.trace), 3)

        # and down ...
        st.update_trace((1.0, 0, 0.0))
        self.assertEqual(len(st.trace), 4)

        st.prune()
        self.assertEqual(len(st.trace), 2)

        # now longer loop
        for i in range(10):
            st.update_trace((1, i, 0))

        for i in range(10):
            st.update_trace((1, 9 - i, 0))

        st.update_trace((2, 0, 0))
        st.update_trace((3, 0, 0))
        self.assertEqual(len(st.trace), 22)

        st.prune()
        self.assertEqual(len(st.trace), 4)

    def test_add_line_to(self):
        t = Trace(1.0)
        t.add_line_to((0, 0, 0))
        t.add_line_to((10, 0, 0))
        self.assertEqual(t.trace[-1], (10, 0, 0))
        self.assertEqual(len(t.trace), 11)
