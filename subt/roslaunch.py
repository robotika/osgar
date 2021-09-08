import os
import signal
import subprocess

from osgar.node import Node


class ROSLaunch(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

        launch_file = config['launch_file']
        extra_args = config.get('args', [])
        cmd = ['roslaunch', launch_file] + extra_args

        self.ros_process = subprocess.Popen(cmd, start_new_session=True)

    def request_stop(self):
        print('roslaunch is shutting down ...')
        os.killpg(self.ros_process.pid, signal.SIGHUP)
        try:
            self.ros_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print('roslaunch still running. Stopping it harder.')
            os.killpg(self.ros_process.pid, signal.SIGTERM)
            try:
                self.ros_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print('roslauncg STILL running. You will likely need to clean-up left-over running modules MANUALLY! "ps ax | grep ros ; kill ..."')
                os.killpg(self.ros_process.pid, signal.SIGKILL)

        super().request_stop()
