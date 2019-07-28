"""
  LoRa - Long Range Radio

  This module handles communication with LoRa devices and automatic re-transmission
  of received messages so that all robots work as re-transmission stations.
  The messages have limited size 40 bytes.

  At the moment driver supports only transmission of "pose2d" messages so the control
  center (and all neighbor robots) know about each other.

  Current protocol uses numbers for DeviceID (1, 2, ... 5) for source identification
  and '|' for data separation. The whole packet is terminated by '\n'. The DeviceID
  is added in front. The message history is collected in reverse order.

  The transmitted messages are echo back with DeviceID prefix. The DeviceID is hardcoded
  in the device firmware (i.e. not controlled by OSGAR). The identification of connected
  device is implemented by transmitting pseudo-random message and parsing its echo.

  Example:
     b'2|5|4|hello' - original message "hello" was sent by device ID=4 and retransmitted
                      by device ID=5 and retransmitted again by device ID=2
"""
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException

ALIVE_MESSAGE = b'alive'


def parse_lora_packet(packet):
    """
       Parse block of binary data. The block starts with integer
       numbers interlaced by '|'. List of transmitter IDs is returned
       with the actual payload data. In the case of missing transmitted
       history header None is returned.
    """
    s = packet.split(b'|')
    try:
        return [int(x) for x in s[:-1]], s[-1].strip()
    except ValueError:
        print('Invalid LoRa packet:', packet)
        return None, packet


def split_lora_buffer(buf):
    if b'\n' not in buf:
        return buf, b''
    i = buf.index(b'\n')
    return buf[i+1:], buf[:i+1]


class LoRa(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.device_id = config.get('device_id')  # None for "autodetect"
        self.min_transmit_dt = timedelta(seconds=10)  # TODO config?
        self.buf = b''
        self.raw = b''  # should be defined by Node
        self.last_transmit = None
        self.recent_packets = []

    def send_data(self, data):
        self.last_transmit = self.publish('raw', data + b'\n')
        return self.last_transmit

    def update(self):
        self.recent_packets = []
        channel = super().update()  # define self.time
        if channel == 'raw':
            print(self.time, "update", channel, self.raw)

            self.buf, packet = split_lora_buffer(self.buf + self.raw)
            while len(packet) > 0:
                self.recent_packets.append(packet)
                addr, data = parse_lora_packet(packet)
                if addr is not None and self.device_id is not None and self.device_id not in addr:
                    print('re-transmit')
                    self.send_data(packet.strip())  # re-transmit data from other robots
                self.buf, packet = split_lora_buffer(self.buf)

        elif channel == 'pose2d':  # accept all types of messages?
            if self.last_transmit is None or self.time > self.last_transmit + self.min_transmit_dt:
                self.send_data(bytes(str(self.pose2d), encoding='ascii'))
        else:
            assert False, channel  # not supported
        return channel

    def run(self):
        start_time = self.send_data(ALIVE_MESSAGE)
        try:
            if self.device_id is None:
                # generate unique messages so that it can be identified
                # among received messages. The device writes all transmitted
                # and received messages. In case of transmitted messages there
                # is my DeviceID.
                unique_cmd = ALIVE_MESSAGE + b'-%d' % start_time.microseconds
                self.send_data(unique_cmd)
                print('Running self-detection... waiting for', unique_cmd)
                while len([packet for packet in self.recent_packets if unique_cmd in packet]) == 0:
                    self.update()
                ref = [packet for packet in self.recent_packets if unique_cmd in packet][0]
                print('Self-identified:', ref)
                addr, data = parse_lora_packet(ref)
                assert addr is not None, ref
                assert len(addr) == 1, ref
                self.device_id = addr[0]
            print('Using device_id', self.device_id)

            while True:
                self.update()
        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
