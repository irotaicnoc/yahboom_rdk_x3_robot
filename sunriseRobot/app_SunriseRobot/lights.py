import time


class Lights:
    def __init__(self, bus):
        self.bus_arg_1 = 0x0d
        self.bus_arg_2_state = 0x07
        self.bus_arg_2_mode = 0x04
        self.stop_cmd = 0
        self.start_cmd = 1
        self.light_effect = 0

        self.bus = bus

        # self.stop()

    def stop(self):
        self.bus.write_byte_data(0x0d, 0x07, self.stop_cmd)
        time.sleep(.05)

    def start(self):
        self.bus.write_byte_data(0x0d, 0x07, self.start_cmd)
        time.sleep(.05)

    def next_effect(self):
        self.light_effect = self.light_effect + 1
        if self.light_effect > 4:
            self.light_effect = 0
            self.bus.write_byte_data(0x0d, 0x07, 0x00)
        else:
            self.bus.write_byte_data(0x0d, 0x04, self.light_effect)
        time.sleep(.05)

    def __del__(self):
        self.stop()
