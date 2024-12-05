import time

import smbus

import args
import global_constants as gc


class Fan:
    def __init__(self, verbose: int = None):
        if verbose is None:
            parameters = args.import_args(
                yaml_path=gc.CONFIG_FOLDER_PATH + 'main_thread_config.yaml',
            )
            self.verbose = parameters['verbose']
        else:
            self.verbose = verbose
        self.bus = smbus.SMBus(0)
        self.start()

    def start(self):
        if self.verbose >= 2:
            print('Starting fan...', end='')
        self.bus.write_byte_data(gc.BUS_ARG_1, gc.BUS_ARG_2_FAN_STATE, gc.FAN_START_CMD)
        time.sleep(.05)
        if self.verbose >= 2:
            print('Done.')

    def stop(self):
        if self.verbose >= 2:
            print('Stopping fan...', end='')
        self.bus.write_byte_data(gc.BUS_ARG_1, gc.BUS_ARG_2_FAN_STATE, gc.FAN_STOP_CMD)
        time.sleep(.05)
        if self.verbose >= 2:
            print('Done.')


if __name__ == '__main__':
    Fan()
