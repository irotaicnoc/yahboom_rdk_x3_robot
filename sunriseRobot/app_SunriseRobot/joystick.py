#!/usr/bin/env python3
# coding=utf-8
import os
import sys
import time
import smbus
import struct

from kill_process import kill_process_
from SunriseRobotLib import SunriseRobot
from ai_agent import AiAgent


# V1.0.4
class Joystick(object):
    def __init__(self, robot: SunriseRobot, js_id=0, debug=False):
        self.__debug = debug
        self.__js_id = int(js_id)
        self.__js_isOpen = False
        self.__ignore_count = 24
        self.__robot = robot

        self.STATE_OK = 0
        self.STATE_NO_OPEN = 1
        self.STATE_DISCONNECT = 2
        self.STATE_KEY_BREAK = 3

        self.__speed_ctrl = 0.7
        self.__speed_x = 0
        self.__speed_y = 0
        self.__speed_z = 0

        self.__hotspot_status = 'inactive'
        self.__ros2_status = 'inactive'
        self.__light_effect = 0
        self.__bus = smbus.SMBus(0)
        # Start with lights turned off
        self.__bus.write_byte_data(0x0d, 0x07, 0x00)

        # autonomous mode
        self.bot_mode_list = ['user_controlled', 'autonomous_tracking']
        self.bot_mode = self.bot_mode_list[0]
        self.tracking_target_list = ['person', 'cat', 'backpack', 'fork', 'knife', 'spoon', 'orange', 'chair', 'remote', 'cell phone']
        self.tracking_target_pos = 0
        self.__last_select_press = 0  # Add timestamp for SELECT button
        self.__select_delay = 5.0  # Minimum seconds between SELECT presses
        self.ai_agent = AiAgent(robot=self.__robot)
        self.ai_agent.deactivate_agent()

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
        if self.__debug:
            print("\n---Joystick DEL---\n")

    # Return joystick state
    def is_Opened(self):
        return self.__js_isOpen

    # Control robot
    def __data_processing(self, name, value):
        if name == "RK1_LEFT_RIGHT":
            if self.bot_mode == 'user_controlled':
                value = -value / 32767
                if self.__debug:
                    print("%s : %.3f" % (name, value))
                self.__speed_y = value * self.__speed_ctrl
                self.__robot.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)

        elif name == 'RK1_UP_DOWN':
            if self.bot_mode == 'user_controlled':
                value = -value / 32767
                if self.__debug:
                    print("%s : %.3f" % (name, value))
                self.__speed_x = value * self.__speed_ctrl
                self.__robot.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)

        elif name == 'RK2_LEFT_RIGHT':
            if self.bot_mode == 'user_controlled':
                value = -value / 32767
                if self.__debug:
                    print("%s : %.3f" % (name, value))
                self.__speed_z = value * 5 * self.__speed_ctrl
                self.__robot.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)

        elif name == 'RK2_UP_DOWN':
            if self.bot_mode == 'user_controlled':
                value = -value / 32767
                if self.__debug:
                    print("%s : %.3f" % (name, value))

        # activate buzzer
        elif name == 'A':
            if self.__debug:
                print(name, ":", value)
            self.__robot.set_beep(value)

        elif name == 'B':
            if self.__debug:
                print(name, ":", value)

        elif name == 'X':
            if self.__debug:
                print(name, ":", value)

        # change light effect
        elif name == 'Y':
            if self.__debug:
                print(name, ":", value)
            if value == 1:
                self.__light_effect = self.__light_effect + 1
                if self.__light_effect > 4:
                    self.__light_effect = 0
                    self.__bus.write_byte_data(0x0d, 0x07, 0x00)
                    time.sleep(.05)
                else:
                    self.__bus.write_byte_data(0x0d, 0x04, self.__light_effect)
                    time.sleep(.05)

        # activate/deactivate hotspot
        elif name == 'L1':
            if self.__debug:
                print(name, ":", value)
            if value == 1:
                if self.__hotspot_status == 'inactive':
                    self.__hotspot_status = 'processing'
                    print("Starting Hotspot...", end='')
                    os.system("sleep 3")
                    os.system("systemctl stop wpa_supplicant")
                    os.system("ip addr flush dev wlan0")
                    os.system("sleep 0.5")
                    os.system("ifconfig wlan0 down")
                    os.system("sleep 1")
                    os.system("ifconfig wlan0 up")
                    os.system("hostapd -B /root/sunriseRobot/hotspot/etc/hostapd.conf")
                    os.system("ifconfig wlan0 192.168.8.88 netmask 255.255.255.0")
                    os.system("systemctl start isc-dhcp-server")
                    print("Done")
                    self.__hotspot_status = 'active'

                elif self.__hotspot_status == 'active':
                    self.__hotspot_status = 'processing'
                    print("Stopping Hotspot...", end='')
                    kill_process_(program_name="hostapd", debug=self.__debug)
                    os.system("systemctl stop isc-dhcp-server")
                    os.system("ip addr flush dev wlan0")
                    os.system("sleep 0.5")
                    os.system("ifconfig wlan0 down")
                    os.system("sleep 1")
                    os.system("ifconfig wlan0 up")
                    os.system("systemctl start wpa_supplicant")
                    print("Done")
                    self.__hotspot_status = 'inactive'

        # activate/deactivate ROS2
        elif name == 'R1':
            if self.bot_mode == 'user_controlled':
                if value == 1:
                    if self.__ros2_status == 'inactive':
                        self.__ros2_status = 'processing'
                        print("Starting ROS2...", end='')
                        os.system("/root/sunriseRobot/app_SunriseRobot/start_ros2.sh")
                        print("Done")
                        self.__ros2_status = 'active'
                    elif self.__ros2_status == 'active':
                        self.__ros2_status = 'processing'
                        print("Stopping ROS2...", end='')
                        kill_process_(program_name="ros2", debug=self.__debug)
                        print("Done")
                        self.__ros2_status = 'inactive'

                if self.__debug:
                    print(name, ":", value)

        # switch between user-controlled mode and autonomous mode
        elif name == 'SELECT':
            if self.__debug:
                print(name, ":", value)
            current_time = time.time()
            if value == 1 and (current_time - self.__last_select_press) >= self.__select_delay:
                print(f"Switching from {self.bot_mode} mode.")
                self.bot_mode = self.bot_mode_list[(self.bot_mode_list.index(self.bot_mode) + 1) % len(self.bot_mode_list)]
                print(f"Switching to {self.bot_mode} mode.")
                self.__last_select_press = current_time
                if self.bot_mode == 'user_controlled':
                    self.ai_agent.deactivate_agent()
                elif self.bot_mode == 'autonomous_tracking':
                    self.ai_agent.activate_agent()

        elif name == 'START':
            if self.__debug:
                print(name, ":", value)
            if value == 1:
                self.__robot.set_beep(1)
            else:
                self.__robot.set_beep(0)

        elif name == 'MODE':
            if self.__debug:
                print(name, ":", value)

        elif name == 'BTN_RK1':
            if self.__debug:
                print(name, ":", value)

        elif name == 'BTN_RK2':
            if self.__debug:
                print(name, ":", value)

        elif name == "L2":
            if self.__debug:
                print("%s : %.3f" % (name, value))

        elif name == "R2":
            if self.__debug:
                print("%s : %.3f" % (name, value))

        # decrease sensibility
        elif name == "L2_1":
            if self.__debug:
                print("%s : %.3f" % (name, value))
            if value == 1:
                self.__speed_ctrl = max(0.0, self.__speed_ctrl - 0.1)

        # increase sensibility
        elif name == "R2_1":
            if self.__debug:
                print("%s : %.3f" % (name, value))
            if value == 1:
                self.__speed_ctrl = min(1.0, self.__speed_ctrl + 0.1)

        elif name == 'WSAD_LEFT_RIGHT':
            value = -value / 32767
            if self.bot_mode == 'user_controlled':
                if self.__debug:
                    print("%s : %.3f" % (name, value))
                if value > 0:
                    self.__robot.set_car_motion(0, self.__speed_ctrl, 0)
                elif value < 0:
                    self.__robot.set_car_motion(0, -self.__speed_ctrl, 0)
                else:
                    self.__robot.set_car_motion(0, 0, 0)

        elif name == 'WSAD_UP_DOWN':
            # in user_controlled mode robot forward/backward
            value = -value / 32767
            if self.bot_mode == 'user_controlled':
                if self.__debug:
                    print("%s : %.3f" % (name, value))
                if value > 0:
                    self.__robot.set_car_motion(self.__speed_ctrl, 0, 0)
                elif value < 0:
                    self.__robot.set_car_motion(-self.__speed_ctrl, 0, 0)
                else:
                    self.__robot.set_car_motion(0, 0, 0)
            # in autonomous_tracking mode cycle through tracking_target_list
            elif self.bot_mode == 'autonomous_tracking':
                if value > 0:
                    self.tracking_target_pos += 1
                    self.tracking_target_pos = self.tracking_target_pos % len(self.tracking_target_list)
                if value < 0:   
                    self.tracking_target_pos -= 1
                    self.tracking_target_pos = self.tracking_target_pos % len(self.tracking_target_list)
                params = {}
                params['target_name'] = self.tracking_target_list[self.tracking_target_pos]
                self.ai_agent.update_params(params=params)

        else:
            pass

    # Handles events for joystick
    def joystick_handle(self):
        if not self.__js_isOpen:
            # if self.__debug:
            #     print('Failed To Open Joystick')
            return self.STATE_NO_OPEN
        try:
            evbuf = self.__jsdev.read(8)
            if evbuf:
                _time, value, _type, number = struct.unpack('IhBB', evbuf)
                func = _type << 8 | number
                name = self.__function_names.get(func)
                # print("evbuf:", _time, value, _type, number)
                # if self.__debug:
                #     print("func:0x%04X, %s, %d" % (func, name, value))
                if name is not None:
                    self.__data_processing(name, value)
                else:
                    if self.__ignore_count > 0:
                        self.__ignore_count = self.__ignore_count - 1
                    if self.__debug and self.__ignore_count == 0:
                        print("Key Value Invalid")
            return self.STATE_OK
        except KeyboardInterrupt:
            if self.__debug:
                print('Key Break Joystick')
            return self.STATE_KEY_BREAK
        except:
            self.__js_isOpen = False
            print('---Joystick Disconnected---')
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
            # if self.__debug:
            #     print('Failed To Open %s' % js)
            return False


if __name__ == '__main__':
    g_debug = False
    if len(sys.argv) > 1:
        if str(sys.argv[1]) == "debug":
            g_debug = True
    print("debug=", g_debug)

    g_robot = SunriseRobot()
    js = Joystick(g_robot, debug=g_debug)
    try:
        while True:
            state = js.joystick_handle()
            if state != js.STATE_OK:
                if state == js.STATE_KEY_BREAK:
                    break
                time.sleep(1)
                js.reconnect()
    except KeyboardInterrupt:
        pass
    del js
