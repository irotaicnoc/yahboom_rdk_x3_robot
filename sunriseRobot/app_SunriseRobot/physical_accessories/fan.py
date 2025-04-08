import time

import smbus


class Fan:
    BUS_ARG_1 = 0x0d
    BUS_ARG_2_STATE = 0x08
    START_CMD = 1
    STOP_CMD = 0

    def __init__(self, verbose: int = 0):
        self.bus = smbus.SMBus(0)
        self.verbose = verbose
        self.start()

    def start(self):
        if self.verbose >= 2:
            print('Starting fan...', end='')
        self.bus.write_byte_data(Fan.BUS_ARG_1, Fan.BUS_ARG_2_STATE, Fan.START_CMD)
        time.sleep(.05)
        if self.verbose >= 2:
            print('Done.')

    def stop(self):
        if self.verbose >= 2:
            print('Stopping fan...', end='')
        self.bus.write_byte_data(Fan.BUS_ARG_1, Fan.BUS_ARG_2_STATE, Fan.STOP_CMD)
        time.sleep(.05)
        if self.verbose >= 2:
            print('Done.')


if __name__ == '__main__':
    Fan()
