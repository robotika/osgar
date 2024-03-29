import logging
import unittest
import unittest.mock

from . import launch


class Test(unittest.TestCase):
    def Xtest_launch(self):
        p = launch.Launch(config={'command': ['true']}, bus=unittest.mock.MagicMock())
        p.start()
        p.join()

    def Xtest_request_stop(self):
        p = launch.Launch(config={'command': ['sleep', '10']}, bus=unittest.mock.MagicMock())
        p.start()
        p.request_stop()
        p.join(0.1)

    def Xtest_join(self):
        with self.assertLogs(level=logging.WARNING) as log:
            p = launch.Launch(config={'command': ['sleep', '10']}, bus=unittest.mock.MagicMock())
            p.start()
            p.join(0.1)
        self.assertEqual(len(log.records), 1)

    def Xtest_shell(self):
        with self.assertLogs(level=logging.WARNING) as log:
            p = launch.Launch(config={'command': 'sleep 10', 'shell': True}, bus=unittest.mock.MagicMock())
            p.start()
            p.join(0.1)
        self.assertEqual(len(log.records), 1)
