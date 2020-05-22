import logging
import signal
import subprocess

g_logger = logging.getLogger(__name__)

class Launch:
    def __init__(self, config, bus):
        self.bus = bus
        self.command = config['command']
        self.shell = config.get('shell', False)

    def start(self):
        self.running = subprocess.Popen(self.command, shell=self.shell)

    def request_stop(self):
        self.bus.shutdown()
        self.running.send_signal(signal.SIGINT)

    def join(self, timeout=None):
        try:
            self.running.wait(timeout)
        except subprocess.TimeoutExpired:
            command = self.command if isinstance(self.command, str) else " ".join(self.command)
            g_logger.warning(f"'{command}' still running, terminating")
            self.running.terminate()
            try:
                self.running.wait(1)
            except subprocess.TimeoutExpired:
                g_logger.warning(f"'{command}' ignoring SIGTERM, killing")
                self.running.kill()
                self.running.wait()
        assert self.running.poll() is not None


if __name__ == "__main__":
    from unittest.mock import MagicMock
    test_launch = Launch(config={'command': ['echo', 'hello world!']}, bus=MagicMock())
    test_launch.start()
    test_launch.join()

    test_request_stop = Launch(config={'command': ['sleep', '10']}, bus=MagicMock())
    test_request_stop.start()
    test_request_stop.request_stop()
    test_request_stop.join(0.1)

    test_join = Launch(config={'command': ['sleep', '10']}, bus=MagicMock())
    test_join.start()
    test_join.join(0.1)
