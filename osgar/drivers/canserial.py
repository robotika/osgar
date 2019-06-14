"""
  Wrapper for CAN-serial communication and control of CAN bridge
"""

import serial
import struct
from threading import Thread
from collections import OrderedDict

from osgar.bus import BusShutdownException


# The CAN message format is specific to CAN bridge used. The first two bytes ("header")
# contain CAN-ID (11bits = 4bits function code and 7bits node ID), RTR (1bit) and the packet
# length (4bits). The bridges uses 0xFF and 0xFE for internal communication control.


CAN_BRIDGE_READY = b'\xfe\x10'  # CAN bridge is ready to accept configuration commands
CAN_BRIDGE_SYNC = b'\xFF'*10    # CAN bridge synchronization bytes
CAN_BRIDGE_START = b'\xfe\x31'  # start bridge

CAN_SPEED = OrderedDict([
                ('10k', b'\xfe\x50'),
                ('20k', b'\xfe\x51'),
                ('50k', b'\xfe\x52'),
                ('125k', b'\xfe\x53'),
                ('250k', b'\xfe\x54'),
                ('500k', b'\xfe\x55'),
                ('800k', b'\xfe\x56'),
                ('1M', b'\xfe\x57')
            ])

# CANopen network management (NMT) communication
# https://en.wikipedia.org/wiki/CANopen#Network_management_(NMT)_protocols
NMT_STATE_BOOT_UP = 0
NMT_STATE_STOPPED = 4
NMT_STATE_OPERATIONAL = 5
NMT_STATE_PREOPERATIONAL = 0x7F

NMT_CMD_GO_TO_OPERATIONAL = 0x01
NMT_CMD_GO_TO_STOPPED = 0x02
NMT_CMD_GO_TO_PREOPERATIONAL = 0x80
NMT_CMD_GO_TO_RESET_NODE = 0x81
NMT_CMD_GO_TO_RESET_COMMUNICATION = 0x82


def is_nmt_message(msg_id):
    # NMT node monitoring (node guarding/heartbeat)
    # 700 + NodeID
    assert (msg_id & 0xF800) == 0, hex(msg_id)  # is not msg_id used for other CAN bridge features?!
    return (msg_id & 0x780) == 0x700  # 4 bit function code, 11 bit node ID


def get_node_id(msg_id):
    return msg_id & 0x7F  # 7 bits address in CANopen


def CAN_packet(msg_id, data):
    header = [(msg_id>>3) & 0xff, (msg_id<<5) & 0xe0 | (len(data) & 0xf)]
    return bytes(header + data)


def parse_header(data):
    assert len(data) >= 2, len(data)
    msg_id = (data[0] << 3) | (data[1] >> 5)
    rtr = (data[1] >> 4) & 0x1  # Remote transmission request
    size = data[1] & 0x0f
    return msg_id, rtr, size


def print_packet(data, dbc = {}):
    assert len(data) >= 2, len(data)
    msg_id, rtr, size = parse_header(data)
    if rtr:
        return [hex(x) for x in data[:2]]
    else:
        assert len(data) == 2 + size, (len(data), 2 + size)
        if msg_id in dbc:
            return hex(msg_id), [hex(x) for x in struct.unpack(dbc[msg_id], data[2:])]
        else:
            return hex(msg_id), [hex(x) for x in data[2:]]


class CANSerial(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.buf = b''

        speed = config.get('speed', '1M')  # default 1Mbit
        if speed not in CAN_SPEED:
            raise ValueError('unsupported speed: {}\nUse:{}'.format(speed, list(CAN_SPEED.keys())))
        self.can_speed_cmd = CAN_SPEED[speed]
        self.is_canopen = config.get('canopen', False)
        self.can_bridge_initialized = False
        self.modules_for_restart = set()

    @staticmethod
    def split_buffer(data):
        # skip 0xFF prefix bytes (CAN bridge control bytes)
        data = data.lstrip(b'\xff')

        if len(data) >= 2:
            # see https://en.wikipedia.org/wiki/CAN_bus
            __, rtr, size = parse_header(data)
            if rtr:
                return data[2:], data[:2]
            elif len(data) >= 2 + size:
                return data[2+size:], data[:2+size]
        return data, b''  # no complete packet available yet

    def send_data(self, module_id, data):
        self.bus.publish('raw', CAN_packet(module_id, data))

    def read_packet(self):
        while True:
            self.buf, packet = self.split_buffer(self.buf)
            if len(packet) > 0:
                msg_id, __, __ = parse_header(packet)
                return msg_id, packet[2:]
            else:
                dt, channel, data = self.bus.listen()
                if channel == 'raw':
                    self.buf += data
                else:
                    print('Ignoring', channel)

    def reset_all_modules(self):
        """
           Reset all modules:
           - broadcast "reset node"
           - make sure every node reaches pre-operational state
           - switch all nodes to operational state
           
           return: list of operational modules
        """
        # reset all nodes
        # addr = 0 means, all nodes must listen
        # data[1] = 0 means, all nodes should act
        self.send_data(0, [NMT_CMD_GO_TO_RESET_NODE, 0])

        ackBootup = [] # Heart Beat 0, bootup, after reset
        ackPreop = [] # HB 127
        ackOp = [] # HB 5

        # The number of modules is unknown, but there is at least one expected.
        # All modules should go through NMT_STATE_BOOT_UP and then 
        # NMT_STATE_PREOPERATIONAL states.
        # If some node fails to switch to preoperation then this loop
        # is going to be infinite! In normal operation finish when
        # all nodes pass through both states, i.e. ==
        while len(ackBootup) == 0 or len(ackBootup) > len(ackPreop):
            msg_id, data = self.read_packet()
            if is_nmt_message(msg_id):
                nodeID = get_node_id(msg_id)
                state = data[0]

                if state == NMT_STATE_BOOT_UP:
                    print("Started module", nodeID)
                    ackBootup.append(nodeID)

                if state == NMT_STATE_PREOPERATIONAL:
                    if nodeID in ackBootup:
                        print("Module", nodeID, "in preoperation.")
                        ackPreop.append(nodeID)
                    else:
                        print("WARNING!!! Module", nodeID, "preop BEFORE bootup!!!")

        print("------- Switch to Operation mode --------")
        self.send_data(0, [NMT_CMD_GO_TO_OPERATIONAL, 0])  # operation mode
        # The same comment as above: if some module refuses to start this loop will
        # be infinite. Otherwise it collects data until all detected modules
        # switch to operational.
        while len(ackPreop) > len(ackOp):
            msg_id, data = self.read_packet()
            if is_nmt_message(msg_id):
                nodeID = get_node_id(msg_id)
                state = data[0]

                if state == NMT_STATE_OPERATIONAL:
                    if nodeID in ackPreop:
                        print("Module", nodeID, "in operation.")
                        ackOp.append(nodeID)
                    else:
                        print("WARNING!!! Module", nodeID, "op BEFORE preop!!!")

        print("collecting some packets ...")
        countHB = 0  # count heart beats
        while countHB < len(ackOp) * 3: # ie approx 3s
            msg_id, data = self.read_packet()
            if is_nmt_message(msg_id):
                countHB += 1
                if countHB % len(ackOp) == 0:
                    print(countHB / len(ackOp), '...')
                assert len(data) == 1, len(data)
                state = data[0]
                if state != NMT_STATE_OPERATIONAL:
                    print('ERROR - module', get_node_id(msg_id), 'data', data)
        return ackOp

    def check_and_restart_module(self, module_id, status):
        if status != NMT_STATE_OPERATIONAL:
            if module_id not in self.modules_for_restart:
                print("RESET", module_id)
                self.bus.publish('raw', CAN_packet(0, [NMT_CMD_GO_TO_RESET_NODE, module_id]))
                self.modules_for_restart.add(module_id)

            elif status == NMT_STATE_PREOPERATIONAL:  # restarted and in preoperation
                print("SWITCH TO OPERATION", module_id)
                self.bus.publish('raw', CAN_packet(0, [NMT_CMD_GO_TO_OPERATIONAL, module_id]))

        elif module_id in self.modules_for_restart:
            print("RUNNING", module_id)
            self.modules_for_restart.remove(module_id)

    def process_packet(self, packet):
        if packet == CAN_BRIDGE_READY:
            self.bus.publish('raw', CAN_BRIDGE_SYNC)
            self.bus.publish('raw', self.can_speed_cmd)
            self.bus.publish('raw', CAN_BRIDGE_START)
            self.can_bridge_initialized = True
            if self.is_canopen:
# HACK!                self.reset_all_modules()  # TODO config
                self.bus.publish('raw', CAN_packet(0, [NMT_CMD_GO_TO_OPERATIONAL, 0]))
            return None

        msg_id, __, __ = parse_header(packet)
        if is_nmt_message(msg_id):
            assert len(packet) == 3, len(packet)
            self.check_and_restart_module(get_node_id(msg_id), packet[2])
        return packet

    def process_gen(self, data):
        self.buf, packet = self.split_buffer(self.buf + data)
        while len(packet) > 0:
            ret = self.process_packet(packet)
            if ret is not None:
                yield ret
            self.buf, packet = self.split_buffer(self.buf)  # i.e. process only existing buffer now

    def slot_raw(self, timestamp, data):
        if len(data) > 0:
            for packet in self.process_gen(data):
                self.bus.publish('can', packet)

    def slot_can(self, timestamp, data):
        if self.can_bridge_initialized:
            # at the moment is can serial just forwarding raw packets
            self.bus.publish('raw', data)
        else:
            print('CAN bridge not initialized yet!')

    def run(self):
        try:
            while True:
                dt, channel, data = self.bus.listen()
                if channel == 'raw':
                    self.slot_raw(dt, data)
                elif channel == 'can':
                    self.slot_can(dt, data)
                else:
                    assert False, channel  # unsupported input channel
        except BusShutdownException:
            if self.is_canopen:
                self.bus.publish('raw', CAN_packet(0, [NMT_CMD_GO_TO_PREOPERATIONAL, 0]))  # pre-operational

    def request_stop(self):
        self.bus.shutdown()


if __name__ == "__main__":
    import argparse
    from osgar.logger import LogReader, lookup_stream_id
    from osgar.lib.serialize import deserialize

    parser = argparse.ArgumentParser(description='Parse CAN stream messages')
    parser.add_argument('logfile', help='filename of stored file')
    parser.add_argument('--stream', help='stream ID or name', default='can.can')
    parser.add_argument('--dbc', help='interpretation of raw data',
                        choices=['spider', 'eduro'])
    args = parser.parse_args()

    dbc = {}
    if args.dbc == 'spider':
        dbc = {
            0x200: 'H',     # status
            0x201: 'HHHH',  # wheels
            0x202: 'HHHH',  # drive status
            0x203: 'HHHH',  # zero steering
            0x204: 'HHBBH'  # user input
        }
    elif args.dbc == 'eduro':
        dbc = {
#            0x80:  # SYNC, no data
            0x181: '<i',    # encoders L
            0x182: '<i',    # encoders R
            #0x187, 0x387, 0x487  # compass, acc
            0x28A: '<H',    # buttons
            0x18B: '<H',    # battery(V)/100.0
        }

    stream_id = lookup_stream_id(args.logfile, args.stream)
    with LogReader(args.logfile, only_stream_id=stream_id) as log:
        for timestamp, stream_id, data in log:
            print(timestamp, print_packet(deserialize(data), dbc))

# vim: expandtab sw=4 ts=4
