import time
import smbus


class Fan:
    def __init__(self):
        self.bus_arg_1 = 0x0d
        self.bus_arg_2 = 0x08
        self.stop_cmd = 0
        self.start_cmd = 1

        self.bus = smbus.SMBus(0)

        self.start()

    def stop(self):
        self.bus.write_byte_data(0x0d, 0x08, self.stop_cmd)
        time.sleep(.05)

    def start(self):
        self.bus.write_byte_data(0x0d, 0x08, self.start_cmd)
        time.sleep(.05)

    def command(self, command: int):
        self.bus.write_byte_data(0x0d, 0x08, command)
        time.sleep(.05)

    def __del__(self):
        self.stop()
