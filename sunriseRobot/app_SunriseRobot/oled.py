#!/usr/bin/env python3
# coding=utf-8

import os
import time
import psutil
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import Adafruit_SSD1306 as SSD

from robot_body import RobotBody

from robot_head import RobotHead


# V1.0.10
class OLED:
    def __init__(self,
                 robot_body: RobotBody,
                 robot_head: RobotHead,
                 i2c_bus=0,
                 clear=False,
                 verbose: int = 0,
                 ):
        self.verbose = verbose
        self.__i2c_bus = i2c_bus
        self.__clear = clear
        self.__top = -2
        self.__x = 0
        self.__oled = None

        self.robot_body = robot_body
        self.robot_head = robot_head

        self.__total_last = 0
        self.__idle_last = 0

        self.__WIDTH = 128
        self.__HEIGHT = 32
        self.__image = Image.new('1', (self.__WIDTH, self.__HEIGHT))
        self.__draw = ImageDraw.Draw(self.__image)
        self.__font = ImageFont.load_default()

    def __del__(self):
        self.clear(True)
        if self.verbose >= 1:
            print('---OLED-DEL---')

    # Initialize OLED, return True on success, False on failure
    def begin(self):
        try:
            self.__oled = SSD.SSD1306_128_32(rst=None, i2c_bus=self.__i2c_bus, gpio=1)
            self.__oled.begin()
            self.__oled.clear()
            self.__oled.display()
            if self.verbose >= 1:
                print('---OLED begin ok!---')
            return True
        except:
            if self.verbose >= 1:
                print('---OLED not found!---')
            return False

    # Clear the display.  Refresh =True Refresh immediately, refresh=False refresh not
    def clear(self, refresh=False):
        self.__draw.rectangle(
            (0, 0, self.__WIDTH, self.__HEIGHT), outline=0, fill=0)
        if refresh:
            try:
                self.refresh()
                return True
            except:
                return False

    # Add characters.  Start_x Start_y indicates the starting point.  Text is the character to be added
    # Refresh =True Refresh immediately, refresh=False refresh not
    def add_text(self, start_x, start_y, text, refresh=False):
        if start_x > self.__WIDTH or start_x < 0 or start_y < 0 or start_y > self.__HEIGHT:
            print('oled text: x, y input error!')
            return
        x = int(start_x + self.__x)
        y = int(start_y + self.__top)
        self.__draw.text((x, y), str(text), font=self.__font, fill=255)
        if refresh:
            self.refresh()

    # line=[1, 4]
    # Write a line of character text.  Refresh =True Refresh immediately, refresh=False refresh not.
    def add_line(self, text, line=1, refresh=False):
        if line < 1 or line > 4:
            print('oled line input error!')
            return
        y = int(8 * (line - 1))
        self.add_text(0, y, text, refresh)

    # Refresh the OLED to display the content
    def refresh(self):
        self.__oled.image(self.__image)
        self.__oled.display()

    # Read the local IP address
    @staticmethod
    def get_local_ip():
        ip = os.popen(
            "/sbin/ifconfig eth0 | grep 'inet' | awk '{print $2}'").read()
        ip = ip[0: ip.find('\n')]
        if ip == '' or len(ip) > 15:
            ip = os.popen(
                "/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
            ip = ip[0: ip.find('\n')]
            if ip == '':
                ip = 'x.x.x.x'
        if len(ip) > 15:
            ip = 'x.x.x.x'
        return ip

    @staticmethod
    def get_ros2_mode_status() -> str:
        ros2_name = 'ros2'
        process_list = psutil.process_iter()
        for process in process_list:
            if ros2_name in process.name():
                return 'Ros Active'
        return 'Ros Inactive'

    def get_controller_mode_status(self) -> tuple:
        status = (
            self.robot_head.robot_mode.replace('_', ' ').title(),
            f'target: {self.robot_head.tracking_target_list[self.robot_head.tracking_target_pos].replace("_", " ").title()}'
        )
        return status

    def get_battery_voltage(self) -> str:
        try:
            voltage = self.robot_body.get_battery_voltage()
            return f'Battery: {voltage:.1f}V'
        except:
            return f'Battery: error'

    # Oled mainly runs functions that are called in a while loop and can be hot-pluggable
    def main_program(self):
        try:
            state = self.begin()
            while state:
                self.clear(refresh=self.__clear)
                str_ip = 'IP:' + self.get_local_ip()
                controller_mode = self.get_controller_mode_status()
                self.add_line(controller_mode[0], line=1)
                if controller_mode[0] == 'Autonomous Tracking':
                    self.add_line(controller_mode[1], line=2)
                else:
                    self.add_line(self.get_ros2_mode_status(), line=2)
                self.add_line(self.get_battery_voltage(), line=3)
                self.add_line(str_ip, line=4)
                # Display image
                self.refresh()
                time.sleep(2)
            return False
        except:
            print('!!!---OLED refresh error---!!!')
            return False
