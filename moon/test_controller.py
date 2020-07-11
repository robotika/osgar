import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.bus import Bus

from moon.controller import SpaceRoboticsChallenge


class EchoController(SpaceRoboticsChallenge):
    pass


echo_data = None

class ControllerTest(unittest.TestCase):
    def test_callback(self):
        global echo_data

        def echo_callback(data):
            global echo_data
            echo_data = data

        config = {}
        bus = Bus(MagicMock(write=MagicMock(return_value=timedelta())))

        echo = EchoController(config, bus=bus.handle('echo'))

        # initialize internal variables, so that wait_for_init() can be skipped
        echo.sim_time = timedelta()
        echo.last_position = [0, 0, 0]
        echo.yaw = 0.0

        tester = bus.handle('tester')
        tester.register("response")
        bus.connect("tester.response", "echo.response")
        bus.connect("echo.request", "tester.request")

        echo.send_request('hello!', echo_callback)
        echo.start()

        _, channel, data = tester.listen()
        self.assertEqual(data, ['0xe3e70682c2094cac629f6fbed82c07cd', 'hello!'])
        tester.publish('response', data)

        echo.bus.sleep(0.1)
        self.assertEqual(echo_data, 'hello!')

        echo.request_stop()
        echo.join()

# vim: expandtab sw=4 ts=4

