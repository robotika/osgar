"""
  Ouster lidar drivers
"""

import json
from threading import Thread
import logging
g_logger = logging.getLogger(__name__)

try:
    from ouster.sdk import open_source, core, sensor
    from ouster.sdk.sensor import ClientTimeout
except ImportError as e:
    g_logger.warning(f"Ouster lidar drivers not installed: {e}")

from osgar.node import Node


class OusterLidarUDP(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('http_request', 'lidar_config')
        self.lidar_url = config["lidar_url"]
        self.config_params = config.get("config_params")
        self.headers = {'Accept': 'application/json', 'Content-Type': 'application/json'}
        self.configuration_done = False
        self.configuration_saved = False
        self.verbose = False

    def send_conf_params(self):
        data = json.dumps(self.config_params).encode('utf-8')
        self.publish("http_request", [self.lidar_url, self.headers, data])

    def request_configuration(self):
        self.publish("http_request", self.lidar_url)

    def process_udp(self, packet):
        pass
        # TODO in some future step

    def on_udp_packet(self, data):
        if not self.configuration_done:
            if self.config_params:
                self.send_conf_params()

            self.request_configuration()
            self.configuration_done = True
            return
        if self.configuration_saved:
            self.process_udp(data)

    def on_response(self, data):
        assert self.configuration_done
        assert not self.configuration_saved  # The configuration should be delivered only once.
        if self.verbose:
            print(data)
        self.publish("lidar_config", data)
        self.configuration_saved = True


class OusterLidar:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        self.bus.register("metadata", "scan3d", "reflectivity")
        self.lidar_ip = config["lidar_ip"]
        lidar_mode = config.get("lidar_mode", "MODE_1024x10")
        assert lidar_mode in ["MODE_512x10", "MODE_1024x10", "MODE_2048x10", "MODE_512x20", "MODE_1024x20"], lidar_mode
        self.lidar_mode = getattr(core.LidarMode, lidar_mode)
        # Azimuth_window: it allows to set the window in the horizontal FOV (mdeg).
        # The origin is at the back of the lidar and continues clockwise.
        self.azimuth_window = config.get("azimuth_window", [0, 360_000])

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        # If we change the parameters, the configuration will take about 10 seconds.
        config = core.SensorConfig()
        config.lidar_mode = self.lidar_mode
        config.azimuth_window = self.azimuth_window

        try:
            sensor.set_config(self.lidar_ip, config, persist=False, udp_dest_auto=True)
        except Exception as e:
            g_logger.error(f"Configuration failed: {e}")
            return

        source = open_source(self.lidar_ip, sensor_idx=0, collate=False)
        info = source.sensor_info[0]
        self.bus.publish("metadata", info.to_json_string())

        scan_iterator = iter(source)
        while self.bus.is_alive():
            try:
                scan_set = next(scan_iterator)
                scan = scan_set[0]

                if scan is not None:
                    if not scan.complete(info.format.column_window):
                        g_logger.warning(f"Scan is incompleted!")
                    range_data = scan.field(core.ChanField.RANGE)
                    self.bus.publish("scan3d", range_data.copy())
                    reflectivity_data = scan.field(core.ChanField.REFLECTIVITY)
                    self.bus.publish("reflectivity", reflectivity_data.copy())

            except StopIteration:
                # Log reading completed
                break

            except ClientTimeout as e:
                g_logger.error(f"Timeout, NO DATA: {e}")
                continue

            except Exception as e:
                g_logger.error(f"Unexpected error: {e}")
                break

    def request_stop(self):
        self.bus.shutdown()
