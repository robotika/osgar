# not so invasive test of Python2 /robot/src/test_*.py files
from osgar.lib import create_load_tests
import os.path
load_tests = create_load_tests(os.path.join(os.dirname(__file__), 'robot', 'src', 'dummy.py'))
