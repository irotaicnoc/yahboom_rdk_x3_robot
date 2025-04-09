#!/usr/bin/env python3
# coding=utf-8
import os
import struct

from controllers.controller_interface import ControllerFunctions


class PS2Controller(object):
    def __init__(self, controller_loop, robot_head, internal_light, gpio_led, js_id: int = 0, verbose: int = 0):
        self.verbose = verbose

        # controller state
        self._js_id = int(js_id)
        self._js_isOpen = False
        self._ignore_count = 24
        self.STATE_OK = 0
        self.STATE_NO_OPEN = 1
        self.STATE_DISCONNECT = 2
        self.STATE_KEY_BREAK = 3
        self.MAX_INPUT_VALUE = 32767

        self.controller_loop = controller_loop

        self.controller_functions = ControllerFunctions(
            controller_loop=controller_loop,
            robot_head=robot_head,
            internal_light=internal_light,
            gpio_led=gpio_led,
            verbose=verbose,
        )

        if self.verbose >= 3:
            print('Available controllers:')
            # Shows the list of controllers, for example: /dev/input/js0
            for fn in os.listdir('/dev/input'):
                if fn.startswith('js'):
                    print('\t/dev/input/%s' % fn)

        # Open the controller device
        try:
            js = '/dev/input/js' + str(self._js_id)
            self._js_dev = open(js, 'rb')
            self._js_isOpen = True
            if self.verbose >= 1:
                print(f'Controller {self._js_id} opened successfully')
            self.controller_loop.connected_controllers += 1
        except:
            self._js_isOpen = False
            if self.verbose >= 1:
                print(f'Failed to open controller {self._js_id}')

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
            # 0x0204: 'AXIS_R2',
            # 0x0205: 'AXIS_L2',
            0x0206: 'AXIS_ARROWS_X',
            0x0207: 'AXIS_ARROWS_Y',
        }

    def __del__(self):
        if self._js_isOpen:
            self._js_dev.close()
        if self.verbose >= 1:
            print(f'Controller {self._js_id} closed successfully')
        self.controller_loop.connected_controllers -= 1

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
            self.controller_functions.axis_left_x(value)

        elif name == 'AXIS_ARROWS_Y':
            value = -value / self.MAX_INPUT_VALUE
            self.controller_functions.axis_left_y(value)

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

        else:
            if self.verbose >= 2:
                print('Unknown button')

    # Handles events for controller
    def event_listener(self):
        if not self._js_isOpen:
            if self.verbose >= 2:
                print('Failed to open controller')
            return self.STATE_NO_OPEN
        try:
            raw_output = self._js_dev.read(8)
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
                        print('Key value invalid')
            return self.STATE_OK
        except KeyboardInterrupt as ki:
            print('Keyboard interrupt')
            print(ki)
            self.controller_loop.connected_controllers -= 1
            return self.STATE_KEY_BREAK
        except Exception as e:
            self._js_isOpen = False
            print('Controller disconnected')
            print(e)
            self.controller_loop.connected_controllers -= 1
            return self.STATE_DISCONNECT

    # reconnect controller
    def reconnect(self):
        try:
            js = '/dev/input/js' + str(self._js_id)
            self._js_dev = open(js, 'rb')
            self._js_isOpen = True
            self._ignore_count = 24
            print(f'Controller with id {self._js_id} opened successfully')
            self.controller_loop.connected_controllers += 1
            return True
        except:
            self._js_isOpen = False
            if self.verbose >= 2:
                print(f'Failed to open controller with id {self._js_id}')
            return False
