#!/usr/bin/env python3
# coding=utf-8
import os
import time
import struct

import utils
from robot_head import RobotHead
from robot_body import RobotBody


# V1.0.4
class Joystick(object):
    def __init__(self, robot_body: RobotBody, robot_head: RobotHead, js_id=0, verbose=False):
        self.verbose = verbose
        self.robot_body = robot_body

        # joystick state
        self.__js_id = int(js_id)
        self.__js_isOpen = False
        self.__ignore_count = 24
        self.STATE_OK = 0
        self.STATE_NO_OPEN = 1
        self.STATE_DISCONNECT = 2
        self.STATE_KEY_BREAK = 3

        self.__speed_x = 0
        self.__speed_y = 0
        self.__speed_z = 0

        # accept only one button input per cooldown
        self.last_select_press = 0  # Add timestamp for SELECT button
        self.select_delay = 5.0  # Minimum seconds between SELECT presses
        self.robot_head = robot_head

        # Find the joystick device.
        print('Joystick Available devices:')
        # Shows the joystick list of the Controller, for example: /dev/input/js0
        for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                print('    /dev/input/%s' % fn)

        # Open the joystick device.
        try:
            js = '/dev/input/js' + str(self.__js_id)
            self.__jsdev = open(js, 'rb')
            self.__js_isOpen = True
            print('---Opening %s Succeeded---' % js)
        except:
            self.__js_isOpen = False
            print('---Failed To Open %s---' % js)

        # Defining Functional List
        # Red LED Mode
        self.__function_names = {
            # BUTTON FUNCTION
            0x0100: 'A',
            0x0101: 'B',
            0x0103: 'X',
            0x0104: 'Y',
            0x0106: 'L1',
            0x0107: 'R1',
            0x0108: 'L2_1',
            0x0109: 'R2_1',
            0x010A: 'SELECT',
            0x010B: 'START',
            0x010D: 'BTN_RK1',
            0x010E: 'BTN_RK2',

            # AXIS FUNCTION
            0x0200: 'RK1_LEFT_RIGHT',
            0x0201: 'RK1_UP_DOWN',
            0x0202: 'RK2_LEFT_RIGHT',
            0x0203: 'RK2_UP_DOWN',
            0x0204: 'R2',
            0x0205: 'L2',
            0x0206: 'WSAD_LEFT_RIGHT',
            0x0207: 'WSAD_UP_DOWN',
        }

        # Green LED Mode
        # self.KEYS = {
        #     # BUTTON FUNCTION
        #     0x0100 : 'A',
        #     0x0101 : 'B',
        #     0x0102 : 'X',
        #     0x0103 : 'Y',
        #     0x0104 : 'L1',
        #     0x0105 : 'R1',
        #     0x0106 : 'SELECT',
        #     0x0107 : 'START',
        #     0x0108 : 'MODE',
        #     0x0109 : 'BTN_RK1',
        #     0x010A : 'BTN_RK2',

        #     # AXIS FUNCTION
        #     0x0200 : 'RK1_LEFT_RIGHT',
        #     0x0201 : 'RK1_UP_DOWN',
        #     0x0202 : 'L2',
        #     0x0203 : 'RK2_LEFT_RIGHT',
        #     0x0204 : 'RK2_UP_DOWN',
        #     0x0205 : 'R2',
        #     0x0206 : 'WSAD_LEFT_RIGHT',
        #     0x0207 : 'WSAD_UP_DOWN',
        # }

    def __del__(self):
        if self.__js_isOpen:
            self.__jsdev.close()
        if self.verbose:
            print('\n---Joystick DEL---\n')

    # Control robot
    def __data_processing(self, name, value):
        if name == 'RK1_LEFT_RIGHT':
            if self.robot_head.robot_mode == 'user_controlled':
                value = -value / 32767
                if self.verbose:
                    print('%s : %.3f' % (name, value))
                self.__speed_y = value * self.robot_head.speed_coefficient
                self.robot_body.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)

        elif name == 'RK1_UP_DOWN':
            if self.robot_head.robot_mode == 'user_controlled':
                value = -value / 32767
                if self.verbose:
                    print('%s : %.3f' % (name, value))
                self.__speed_x = value * self.robot_head.speed_coefficient
                self.robot_body.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)

        elif name == 'RK2_LEFT_RIGHT':
            if self.robot_head.robot_mode == 'user_controlled':
                value = -value / 32767
                if self.verbose:
                    print('%s : %.3f' % (name, value))
                self.__speed_z = value * self.robot_head.speed_coefficient * self.robot_head.steer_speed_proportion
                self.robot_body.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)

        elif name == 'RK2_UP_DOWN':
            if self.robot_head.robot_mode == 'user_controlled':
                value = -value / 32767
                if self.verbose:
                    print('%s : %.3f' % (name, value))

        # activate buzzer
        elif name == 'A':
            if self.verbose:
                print(name, ':', value)
            self.robot_body.set_beep(value)

        # elif name == 'B':
        #     if self.verbose:
        #         print(name, ':', value)
        #
        # elif name == 'X':
        #     if self.verbose:
        #         print(name, ':', value)

        # change light effect
        elif name == 'Y':
            if self.verbose:
                print(name, ':', value)
            if value == 1:
                self.robot_head.next_light_effect()

        # activate/deactivate hotspot
        elif name == 'L1':
            if self.verbose:
                print(name, ':', value)
            if value == 1:
                if self.robot_head.hotspot_status == 'inactive':
                    self.robot_head.hotspot_status = 'processing'
                    utils.activate_hotspot(verbose=self.verbose)
                    self.robot_head.hotspot_status = 'active'

                elif self.robot_head.hotspot_status == 'active':
                    self.robot_head.hotspot_status = 'processing'
                    utils.deactivate_hotspot(verbose=self.verbose)
                    self.robot_head.hotspot_status = 'inactive'

        # activate/deactivate ROS2
        elif name == 'R1':
            if self.robot_head.robot_mode == 'user_controlled':
                if value == 1:
                    if self.robot_head.ros2_status == 'inactive':
                        self.robot_head.ros2_status = 'processing'
                        utils.activate_ros2(verbose=self.verbose)
                        self.robot_head.ros2_status = 'active'
                    elif self.robot_head.ros2_status == 'active':
                        self.robot_head.ros2_status = 'processing'
                        utils.deactivate_ros2(verbose=self.verbose)
                        self.robot_head.ros2_status = 'inactive'

                if self.verbose:
                    print(name, ':', value)

        # switch between user-controlled mode and autonomous mode
        elif name == 'SELECT':
            if self.verbose:
                print(name, ':', value)
            current_time = time.time()
            if value == 1 and (current_time - self.last_select_press) >= self.select_delay:
                self.last_select_press = current_time
                self.robot_head.next_mode()

        elif name == 'START':
            if self.verbose:
                print(name, ':', value)
            if value == 1:
                self.robot_body.set_beep(1)
            else:
                self.robot_body.set_beep(0)

        # elif name == 'MODE':
        #     if self.verbose:
        #         print(name, ':', value)
        #
        # elif name == 'BTN_RK1':
        #     if self.verbose:
        #         print(name, ':', value)
        #
        # elif name == 'BTN_RK2':
        #     if self.verbose:
        #         print(name, ':', value)
        #
        # elif name == 'L2':
        #     if self.verbose:
        #         print('%s : %.3f' % (name, value))
        #
        # elif name == 'R2':
        #     if self.verbose:
        #         print('%s : %.3f' % (name, value))

        # decrease sensibility
        elif name == 'L2_1':
            if self.verbose:
                print('%s : %.3f' % (name, value))
            if value == 1:
                self.robot_head.decrease_speed_coefficient()

        # increase sensibility
        elif name == 'R2_1':
            if self.verbose:
                print('%s : %.3f' % (name, value))
            if value == 1:
                self.robot_head.increase_speed_coefficient()

        elif name == 'WSAD_LEFT_RIGHT':
            value = -value / 32767
            if self.robot_head.robot_mode == 'user_controlled':
                if self.verbose:
                    print('%s : %.3f' % (name, value))
                if value > 0:
                    self.robot_body.set_car_motion(0, self.robot_head.speed_coefficient, 0)
                elif value < 0:
                    self.robot_body.set_car_motion(0, -self.robot_head.speed_coefficient, 0)
                else:
                    self.robot_body.set_car_motion(0, 0, 0)

        elif name == 'WSAD_UP_DOWN':
            # in user_controlled mode robot forward/backward
            value = -value / 32767
            if self.robot_head.robot_mode == 'user_controlled':
                if self.verbose:
                    print('%s : %.3f' % (name, value))
                if value > 0:
                    self.robot_body.set_car_motion(self.robot_head.speed_coefficient, 0, 0)
                elif value < 0:
                    self.robot_body.set_car_motion(-self.robot_head.speed_coefficient, 0, 0)
                else:
                    self.robot_body.set_car_motion(0, 0, 0)
            # in autonomous_tracking mode cycle through tracking_target_list
            elif self.robot_head.robot_mode == 'autonomous_tracking':
                if value > 0:
                    self.robot_head.next_target()
                if value < 0:   
                    self.robot_head.previous_target()

        else:
            pass

    # Handles events for joystick
    def joystick_handle(self):
        if not self.__js_isOpen:
            # if self.verbose:
            #     print('Failed To Open Joystick')
            return self.STATE_NO_OPEN
        try:
            evbuf = self.__jsdev.read(8)
            if evbuf:
                _time, value, _type, number = struct.unpack('IhBB', evbuf)
                func = _type << 8 | number
                name = self.__function_names.get(func)
                # print('evbuf:', _time, value, _type, number)
                # if self.verbose:
                #     print('func:0x%04X, %s, %d' % (func, name, value))
                if name is not None:
                    self.__data_processing(name, value)
                else:
                    if self.__ignore_count > 0:
                        self.__ignore_count = self.__ignore_count - 1
                    if self.verbose and self.__ignore_count == 0:
                        print('Key Value Invalid')
            return self.STATE_OK
        except KeyboardInterrupt as ki:
            print('Key Break Joystick')
            print(ki)
            return self.STATE_KEY_BREAK
        except Exception as e:
            self.__js_isOpen = False
            print('---Joystick Disconnected---')
            print(e)
            return self.STATE_DISCONNECT

    # reconnect Joystick
    def reconnect(self):
        try:
            js = '/dev/input/js' + str(self.__js_id)
            self.__jsdev = open(js, 'rb')
            self.__js_isOpen = True
            self.__ignore_count = 24
            print('---Opening %s Succeeded---' % js)
            return True
        except:
            self.__js_isOpen = False
            # if self.verbose:
            #     print('Failed To Open %s' % js)
            return False
