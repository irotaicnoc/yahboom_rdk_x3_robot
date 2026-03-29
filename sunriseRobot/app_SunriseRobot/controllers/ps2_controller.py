#!/usr/bin/env python3
# coding=utf-8
import os
import struct

import utils
import global_constants as gc


class PS2Controller(object):
    def __init__(self, controller_functions, controller_id: int = 0, verbose: int = 0):
        self.verbose = verbose

        # controller state
        self.controller_id = int(controller_id)
        self._is_connected = False
        self._ignore_count = 24
        self.MAX_INPUT_VALUE = 32767

        self.controller_functions = controller_functions

        if self.verbose >= 3:
            print('Available controllers:')
            # Shows the list of controllers, for example: /dev/input/js0
            for fn in os.listdir('/dev/input'):
                if fn.startswith('js'):
                    print('\t/dev/input/%s' % fn)

        # Open the controller device
        try:
            controller_path = '/dev/input/js' + str(self.controller_id)
            self._controller = open(controller_path, 'rb')
            self._is_connected = True
            if self.verbose >= 1:
                print(f'Controller {self.controller_id} opened successfully')
            self.controller_functions.connected(controller_id=self.controller_id)
        except:
            self._is_connected = False
            if self.verbose >= 1:
                print(f'Failed to open controller {self.controller_id}')

        # Defining Functional List
        self._function_names = {
            # BUTTON FUNCTIONS
            0x0100: 'BUTTON_A',
            0x0101: 'BUTTON_B',
            0x0103: 'BUTTON_X',
            0x0104: 'BUTTON_Y',
            0x0106: 'BUTTON_L1',
            0x0107: 'BUTTON_R1',
            0x0108: 'BUTTON_L2',
            0x0109: 'BUTTON_R2',
            0x010A: 'BUTTON_SELECT',
            0x010B: 'BUTTON_START',
            0x010D: 'BUTTON_ROCKER_LEFT',
            0x010E: 'BUTTON_ROCKER_RIGHT',

            # AXIS FUNCTIONS
            0x0200: 'AXIS_ROCKER_LEFT_X',
            0x0201: 'AXIS_ROCKER_LEFT_Y',
            0x0202: 'AXIS_ROCKER_RIGHT_X',
            0x0203: 'AXIS_ROCKER_RIGHT_Y',
            0x0204: 'AXIS_R2',
            0x0205: 'AXIS_L2',
            0x0206: 'AXIS_ARROWS_X',
            0x0207: 'AXIS_ARROWS_Y',
        }

    def __del__(self):
        if self._is_connected:
            self._controller.close()
            self._is_connected = False
            self.controller_functions.disconnected(controller_id=self.controller_id)
        if self.verbose >= 1:
            print(f'Controller {self.controller_id} closed successfully')

    def standardize_signal(self, name: str, value):
        if self.verbose >= 3:
            if 'AXIS' in name:
                print(f'{name}: {value:.2f}')
            else:
                print(f'{name}: {value}')

        if name == 'AXIS_ROCKER_LEFT_X':
            value = -value / self.MAX_INPUT_VALUE
            self.controller_functions.axis_left_x(value)

        elif name == 'AXIS_ROCKER_LEFT_Y':
            value = -value / self.MAX_INPUT_VALUE
            self.controller_functions.axis_left_y(value)

        elif name == 'AXIS_ROCKER_RIGHT_X':
            value = -value / self.MAX_INPUT_VALUE
            self.controller_functions.axis_right_x(value)

        elif name == 'AXIS_ROCKER_RIGHT_Y':
            value = -value / self.MAX_INPUT_VALUE
            self.controller_functions.axis_right_y(value)

        elif name == 'AXIS_ARROWS_X':
            value = -value / self.MAX_INPUT_VALUE
            self.controller_functions.axis_arrows_x(value)

        elif name == 'AXIS_ARROWS_Y':
            value = -value / self.MAX_INPUT_VALUE
            self.controller_functions.axis_arrows_y(value)

        elif name == 'BUTTON_A':
            self.controller_functions.button_south(value)

        elif name == 'BUTTON_B':
            self.controller_functions.button_east(value)

        elif name == 'BUTTON_X':
            self.controller_functions.button_west(value)

        elif name == 'BUTTON_Y':
            self.controller_functions.button_north(value)

        elif name == 'BUTTON_L1':
            self.controller_functions.button_l1(value)

        elif name == 'BUTTON_R1':
            self.controller_functions.button_r1(value)

        elif name == 'BUTTON_L2':
            self.controller_functions.button_l2(value)

        elif name == 'BUTTON_R2':
            self.controller_functions.button_r2(value)

        elif name == 'BUTTON_SELECT':
            self.controller_functions.button_select(value)

        elif name == 'BUTTON_START':
            self.controller_functions.button_start(value)

        elif name == 'BUTTON_ROCKER_LEFT':
            self.controller_functions.button_rocker_left(value)

        elif name == 'BUTTON_ROCKER_RIGHT':
            self.controller_functions.button_rocker_right(value)

        elif name == 'AXIS_L2' or name == 'AXIS_R2':
            # ignore this command but catch it in this branch, otherwise it will generate an error and cause the
            # controller to be continuously disconnected and reconnected. This is caused by L2 and R2 generating both
            # a button and an axis event (sometimes more than one axis events). This is a bug in the controller.
            pass

        else:
            self.controller_functions.unknown_input(name, value)

    # Handles events for controller
    def event_listener(self):
        if not self._is_connected:
            if self.verbose >= 2:
                print('Failed to open controller')
            return gc.STATE_NO_OPEN
        try:
            raw_output = self._controller.read(8)
            if raw_output:
                _time, value, _type, number = struct.unpack('IhBB', raw_output)
                func = _type << 8 | number
                name = self._function_names.get(func)
                # print(f'_time: {_time}, name: {name}, value: {value}')
                if name is not None:
                    self.standardize_signal(name, value)
                else:
                    if self._ignore_count > 0:
                        self._ignore_count = self._ignore_count - 1
                    if self.verbose >= 2 and self._ignore_count == 0:
                        print(f'The received controller input {func}, is not in "_function_names"')
            return gc.STATE_OK
        except KeyboardInterrupt as ki:
            self._is_connected = False
            print('Keyboard interrupt')
            print(ki)
            self.controller_functions.disconnected(controller_id=self.controller_id)
            return gc.STATE_KEY_BREAK
        except Exception as e:
            self._is_connected = False
            utils.print_exception(exception=e, message='Controller disconnected due to error')
            self.controller_functions.disconnected(controller_id=self.controller_id)
            return gc.STATE_DISCONNECT

    # reconnect controller
    def reconnect(self):
        try:
            controller_path = '/dev/input/js' + str(self.controller_id)
            self._controller = open(controller_path, 'rb')
            self._is_connected = True
            self._ignore_count = 24
            print(f'Controller with id {self.controller_id} opened successfully')
            self.controller_functions.connected(controller_id=self.controller_id)
            return True
        except:
            self._is_connected = False
            if self.verbose >= 2:
                print(f'Failed to open controller with id {self.controller_id}')
            return False
