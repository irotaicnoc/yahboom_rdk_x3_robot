import time


class Fan:
    def __init__(self, bus):
        print('Creating Fan.')
        self.bus_arg_1 = 0x0d
        self.bus_arg_2 = 0x08
        self.stop_cmd = 0
        self.start_cmd = 1
        self.bus = bus
        print(f'bus: {bus}')

    def stop(self):
        self.bus.write_byte_data(self.bus_arg_1, self.bus_arg_2, self.stop_cmd)
        time.sleep(.05)

    def start(self):
        print('Starting fan... ', end='')
        self.bus.write_byte_data(self.bus_arg_1, self.bus_arg_2, self.start_cmd)
        time.sleep(.05)
        print('Done.')

    def command(self, command: int):
        self.bus.write_byte_data(self.bus_arg_1, self.bus_arg_2, command)
        time.sleep(.05)

    def __del__(self):
        self.stop()
