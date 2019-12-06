"""
  Communication module for vESC - open source BLDC motor driver
  https://www.vandaelectronics.com/collections/electronic-speed-controls/products/vesc
  https://github.com/vedderb/bldc

  There are several ways how to communicate with the device and here
  we work with CAN bus. The motors uses extended addresing mode (24bits)
  and combines command with device identification into the address.

  At the moment only one type of message is decoded and that is "read buffer".
  The device responds with provided "last_message_id" encoded in the address.
  As we need to communicate with four BLDC motors we abuse this "last_message_id"
  for motor identification, so to query motor #1 we set "last_message_id" to 1 etc.
  The reponse is then collection of several CAN messages with address 0x5nn, where nn
  is the motor number, i.e. 0x501 for motor #1.

  Each 0x5nn message contains offset byte in final buffer and seven data bytes (CAN messages
  are limited to 8 bytes only). 

  The transmission is terminated with 0x7nn message, which carries control chechsum and
  total length of already transmitted data.

  Note that the status structure varies among versions and now we are experimenting with
  version 3.40 only.

  For more info see:
     https://robotika.cz/articles/reviews/vesc
"""

import struct
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException


def draw_diff(arr):
    import matplotlib.pyplot as plt
    t = [a[0] for a in arr]
    values = [a[1:] for a in arr]

    line = plt.plot(t, values, '-o', linewidth=2)

    plt.xlabel('time (s)')
    plt.show()


class MotorDriverVESC(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('can')
        self.debug_arr = []
        self.prev = {}
        self.verbose = False  # TODO move into Node
        self.packet = [[], [], [], []]
        self.tachometer = [None, None, None, None]  # for motors #1, #2, #3 and #4

    def update(self):
        channel = super().update()
        assert channel == 'can', channel
        msg_id, data, flags = self.can  # via update()
        if (msg_id & 0xF00) == 0x500:
            motor_index = (msg_id & 0xFF) - 1  # reindexed motors from #1 to array index 0
            assert data[0] % 7 == 0, data[0]
            self.packet[motor_index].extend(data[1:])
            # the structure for version 3.40 has 59 bytes and looks like:
            # https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L166-L185
            """
     case COMM_GET_VALUES:
     ind = 0;
 0   send_buffer[ind++] = COMM_GET_VALUES;
 1   buffer_append_float16(send_buffer, mc_interface_temp_fet_filtered(), 1e1, &ind);
 3   buffer_append_float16(send_buffer, mc_interface_temp_motor_filtered(), 1e1, &ind);
 5   buffer_append_float32(send_buffer, mc_interface_read_reset_avg_motor_current(), 1e2, &ind);
 9   buffer_append_float32(send_buffer, mc_interface_read_reset_avg_input_current(), 1e2, &ind);
13   buffer_append_float32(send_buffer, mc_interface_read_reset_avg_id(), 1e2, &ind);
17   buffer_append_float32(send_buffer, mc_interface_read_reset_avg_iq(), 1e2, &ind);
21   buffer_append_float16(send_buffer, mc_interface_get_duty_cycle_now(), 1e3, &ind);
23   buffer_append_float32(send_buffer, mc_interface_get_rpm(), 1e0, &ind);
27   buffer_append_float16(send_buffer, GET_INPUT_VOLTAGE(), 1e1, &ind);
29   buffer_append_float32(send_buffer, mc_interface_get_amp_hours(false), 1e4, &ind);
33   buffer_append_float32(send_buffer, mc_interface_get_amp_hours_charged(false), 1e4, &ind);
37   buffer_append_float32(send_buffer, mc_interface_get_watt_hours(false), 1e4, &ind);
41   buffer_append_float32(send_buffer, mc_interface_get_watt_hours_charged(false), 1e4, &ind);
45   buffer_append_int32(send_buffer, mc_interface_get_tachometer_value(false), &ind);
49   buffer_append_int32(send_buffer, mc_interface_get_tachometer_abs_value(false), &ind);
53   send_buffer[ind++] = mc_interface_get_fault();
54   buffer_append_float32(send_buffer, mc_interface_get_pid_pos_now(), 1e6, &ind);
58   send_buffer[ind++] = app_get_configuration()->controller_id;
59
commands_send_packet(send_buffer, ind);
"""
            # the "tachometer" data are stored as 32bit signed integer starting
            # from 45th byte. Because the data blocks contain 7 bytes we are
            # interested in block with "address pointer" set to 42.
            # Then we do not have to re-contruct all four independent bufferes
            # but only "pick" the interesting sub-packet. Yes, there is a danger
            # that checksum of the whole buffer was not valid!
            if data[0] == 42:  # optimization - pick only this packet part
                b = bytes(data[1:])
                prev = self.tachometer[motor_index]
                self.tachometer[motor_index] = struct.unpack_from('>i', b, 3)[0]
                if prev != self.tachometer[motor_index] and self.verbose:
                    print(self.time, hex(msg_id), self.tachometer[motor_index])
                tmp = [None] * 4
                tmp[motor_index] = self.tachometer[motor_index]
                if self.verbose:
                    self.debug_arr.append([self.time.total_seconds()] + tmp)
        if (msg_id & 0xF00) == 0x700:
            motor_index = (msg_id & 0xFF) - 1  # reindexed motors from #1 to array index 0
            if len(self.packet[motor_index]) == 19:  # version
                print(self.time, self.packet[motor_index])
            self.packet[motor_index] = []
        if self.verbose > 1:
            print(self.time, hex(msg_id), list(data), flags)
        return channel

    def run(self):
        try:
            self.update()  # initialize self.time
            while True:
                # example to query all variables
                # COMM_GET_VALUES=4
                for rx_buffer_last_id in [1, 2, 3, 4]:
                    self.publish('can', [0x800 + rx_buffer_last_id, bytes([rx_buffer_last_id, 0, 4]), 1])

                start_time = self.time
                count = 0
                while self.time - start_time < timedelta(seconds=0.1):
                    self.update()
                    msg_id = self.can[0]
                    if (msg_id & 0xF00) == 0x700:
                        count += 1
                        if count >= 4:
                            break
                if count != 4 and self.verbose:
                    print(self.time, "vESC: incomplete loop")

        except BusShutdownException:
            pass


    def draw(self):
        """
        Debug Draw
        """
        draw_diff(self.debug_arr)

# vim: expandtab sw=4 ts=4
