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

  I/O:
  This driver can now parse commands for particular device in the form:
    <device_id>:<cmd>:<hash>
  The <device_id> has to match in order to publish <cmd>. <hash> is used only for distinction
  of messages, and the plan is to use seconds since control center start.
  Broadcast command for all robots is realized by <device_id> = 0.

"""
from datetime import timedelta
from ast import literal_eval

from osgar.node import Node
from osgar.bus import BusShutdownException

ALIVE_MESSAGE = b'alive'
ALLOWED_DEVICE_IDS = [1, 2, 3, 4, 5, 6]


def parse_lora_packet(packet):
    """
       Parse block of binary data. The block starts with integer
       numbers interlaced by '|'. List of transmitter IDs is returned
       with the actual payload data. In the case of missing transmitted
       history header None is returned.
    """
    s = packet.split(b'|')
    try:
        addr, data = [int(x) for x in s[:-1]], s[-1].strip()
    except ValueError:
        return None, packet

    if len(addr) < 1 or not min([a in ALLOWED_DEVICE_IDS for a in addr]):
        return None, packet
    return addr, data


def split_lora_buffer(buf):
    if b'\n' not in buf:
        return buf, b''
    i = buf.index(b'\n')
    return buf[i+1:], buf[:i+1]


def parse_my_cmd(my_id, data):
    s = data.split(b':')
    try:
        destination_id = int(s[0])
        if destination_id == my_id or destination_id == 0:
            return s[1]
    except:
        pass
    return None


def draw_lora_positions(arr):
    """
    Draw positions of tripples:
        (time, ID, (x. y, heading))
    """
    import matplotlib.pyplot as plt
    t = [a[0] for a in arr]
    robot_ids = sorted(list(set([a[1] for a in arr])))
    print('Robot IDs', robot_ids)

    for robot_id in robot_ids:
        x = [a[2][0]/1000.0 for a in arr if a[1] == robot_id]
        y = [a[2][1]/1000.0 for a in arr if a[1] == robot_id]
        line = plt.plot(x, y, '-o', linewidth=2, label='Robot #%d' % robot_id)

#    plt.xlabel('time (s)')
    plt.axes().set_aspect('equal', 'datalim')
    plt.legend()
    plt.show()


class LoRa(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('raw', 'cmd', 'robot_status', 'artf', 'artf_xyz')
        self.device_id = config.get('device_id')  # None for "autodetect"
        self.init_sleep = config.get('sleep')  # needed for Windows on start
        self.min_transmit_dt = timedelta(seconds=10)  # TODO config?
        self.buf = b''
        self.raw = b''  # should be defined by Node
        self.last_transmit = None
        self.recent_packets = []
        self.verbose = config.get('verbose', False)
        self.debug_robot_poses = []

    def send_data(self, data):
        self.last_transmit = self.publish('raw', data + b'\n')
        return self.last_transmit

    def update(self):
        self.recent_packets = []
        channel = super().update()  # define self.time
        if channel == 'raw':
            if self.verbose:
                print(self.time, "update", channel, self.raw)

            self.buf, packet = split_lora_buffer(self.buf + self.raw)
            while len(packet) > 0:
                self.recent_packets.append(packet)
                addr, data = parse_lora_packet(packet)
                if addr is not None and data.startswith(b'['):
                    arr = literal_eval(data.decode('ascii'))
                    if len(arr) == 3:
                        pose2d = arr
                        self.publish('robot_status', [addr[-1], pose2d, b'running'])  # TODO read it from robot
                    else:
                        assert len(arr) == 4, arr  # artifact, x, y z
                        self.publish('artf', [addr[-1], arr])
                        self.publish('artf_xyz', [arr])  # publish also standard "list" of detected artifacts
                    if self.verbose:
                        self.debug_robot_poses.append((self.time.total_seconds(), addr[-1], pose))
                cmd = parse_my_cmd(self.device_id, data)
                if cmd is not None:
                    self.publish('cmd', cmd)
                if addr is not None and self.device_id is not None and self.device_id not in addr:
                    if self.verbose:
                        print('re-transmit')
                    self.send_data(packet.strip())  # re-transmit data from other robots
                self.buf, packet = split_lora_buffer(self.buf)

        elif channel == 'pose2d':
            if self.last_transmit is None or self.time > self.last_transmit + self.min_transmit_dt:
                self.send_data(bytes(str(self.pose2d), encoding='ascii'))
        elif channel == 'cmd':
            assert len(self.cmd) == 2, self.cmd
            self.send_data(b'%d:%s:%d' % (self.cmd[0], self.cmd[1], int(self.time.total_seconds())))
        elif channel == 'artf':
            # send data as they are, ignore transmit time, ignore transmit failure
            for artf_item in self.artf:
                self.send_data(bytes(str(artf_item), encoding='ascii'))
        else:
            assert False, channel  # not supported
        return channel

    def run(self):
        start_time = self.send_data(ALIVE_MESSAGE)
        if self.init_sleep is not None:
            self.bus.sleep(self.init_sleep)  # workaround for reset on COM open under Windows
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

    def draw(self):
        """
        Debug Draw
        """
        draw_lora_positions(self.debug_robot_poses)


# vim: expandtab sw=4 ts=4
