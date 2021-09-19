import atexit
import os
import signal
import subprocess

from osgar.node import Node


class ROSLaunch(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register()

        launch_file = config['launch_file']
        extra_args = config.get('args', [])
        cmd = ['roslaunch', launch_file] + extra_args

        self.ros_process = subprocess.Popen(cmd, start_new_session=True)

        atexit.register(self.quit)

    def request_stop(self):
        self.quit()
        super().request_stop()

    def quit(self):
        print('roslaunch is shutting down ...')
        try:
            os.killpg(self.ros_process.pid, signal.SIGHUP)
        except ProcessLookupError:
            # The process group no longer exists. It may have even be stopped by
            # request_stop()
            return
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
