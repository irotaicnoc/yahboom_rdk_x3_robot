import time


class Lights:
    def __init__(self, bus):
        print('Creating Lights.')
        self.bus_arg_1 = 0x0d
        self.bus_arg_2_state = 0x07
        self.bus_arg_2_mode = 0x04
        self.stop_cmd = 0
        self.start_cmd = 1
        self.current_effect_cmd = 0

        self.bus = bus

        # self.stop()

    def stop(self):
        self.bus.write_byte_data(self.bus_arg_1, self.bus_arg_2_state, self.stop_cmd)
        time.sleep(.05)

    def start(self):
        self.bus.write_byte_data(self.bus_arg_1, self.bus_arg_2_state, self.start_cmd)
        time.sleep(.05)

    def set_effect(self, effect_cmd: int = None):
        if effect_cmd is None:
            effect_cmd = self.current_effect_cmd
        if effect_cmd == self.stop_cmd:
            self.stop()
        else:
            self.bus.write_byte_data(self.bus_arg_1, self.bus_arg_2_mode, effect_cmd)
            time.sleep(.05)

    def next_effect(self):
        self.current_effect_cmd = self.current_effect_cmd + 1
        if self.current_effect_cmd > 4:
            self.current_effect_cmd = 0

        self.set_effect()

    def __del__(self):
        self.stop()
