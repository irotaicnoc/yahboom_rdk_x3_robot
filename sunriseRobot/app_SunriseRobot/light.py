import time

import smbus

import global_constants as gc


class Light:
    def __init__(self, verbose: int = 0):
        self.current_state = gc.LIGHT_STOP_CMD
        self.bus = smbus.SMBus(0)
        self.verbose = verbose
        self.stop()

    def stop(self):
        if self.verbose >= 2:
            print('Turning off lights...', end='')
        self.bus.write_byte_data(gc.BUS_ARG_1, gc.BUS_ARG_2_LIGHT_STATE, gc.LIGHT_STOP_CMD)
        self.current_state = gc.LIGHT_STOP_CMD
        time.sleep(.05)
        if self.verbose >= 2:
            print('Done.')

    def set_state(self, state: int):
        if state == gc.LIGHT_STOP_CMD:
            self.stop()
        else:
            if self.verbose >= 2:
                print(f'Setting light effect {state}...', end='')
            self.bus.write_byte_data(gc.BUS_ARG_1, gc.BUS_ARG_2_LIGHT_MODE, state)
            self.current_state = state
            time.sleep(.05)
            if self.verbose >= 2:
                print('Done.')

    def next_light_effect(self):
        self.set_state(gc.LIGHT_EFFECT_CMD_LIST[
            (gc.LIGHT_EFFECT_CMD_LIST.index(self.current_state) + 1) % len(gc.LIGHT_EFFECT_CMD_LIST)
        ])

    def __del__(self):
        self.stop()
