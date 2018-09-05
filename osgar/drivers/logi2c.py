"""
  Wrapper & timestamper for I2C I/O
"""
# primarily developed for RaspberryPI on boat Marina 2.0

from threading import Thread

try:
    import smbus
except:
    import logging
    logging.warning('Package "smbus" is not available!')
    class smbus:
        class SMBus:
            pass

from osgar.bus import BusShutdownException


class LogI2C(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        if 'port' in config:
            self.com = smbus.SMBus(config['port'])
        else:
            self.com = None
        self.bus = bus

    def run(self):
        try:
            while True:
                __, __, data = self.bus.listen()
                assert len(data) == 4, data
                addr, rw, reg, len_or_data = data
                assert rw in ['R', 'W'], rw
                if rw == 'R':
                    try:
                        received = self.com.read_i2c_block_data(addr, reg, len_or_data)
                    except OSError as e:
                        print(e)
                        with self.bus.logger.lock:
                            self.bus.logger.write(0, bytes(str(e), encoding='ascii'))
                        received = self.com.read_i2c_block_data(addr, reg, len_or_data)
                    self.bus.publish('i2c', [addr, reg, received])  # TODO prefix
                else:  # 'W'
                    self.com.write_i2c_block_data(addr, reg, len_or_data)
                    # TODO send something to the system?
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


# vim: expandtab sw=4 ts=4
