#!/usr/bin/env python3
# coding=utf-8
import time
import warnings

import utils


class ControllerFunctions(object):
    def __init__(self,
                 controller_loop,
                 robot_head,
                 internal_light,
                 gpio_led,
                 verbose: int = 0,
                 ):

        self.controller_loop = controller_loop
        self.robot_head = robot_head
        self.internal_light = internal_light
        self.gpio_led = gpio_led
        self.verbose = verbose

        # accept only one button input per cooldown
        self.last_select_press = 0  # Add timestamp for SELECT button
        self.last_start_press = 0  # Add timestamp for START button
        self.BUTTON_COOLDOWN = 5.0  # Minimum seconds between button presses

    # value is True or False for buttons
    # value is a float in range [-1, 1] for axes
    # for arrows 'value' is a float in range [-1, 1], but it can only assume the values -1, 0 or 1
    def axis_left_x(self, value: float):
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.controller_loop.speed_y = value * self.robot_head.speed_coefficient
        elif self.robot_head.robot_mode == 'user_control_arm':
            print(f'servo 1 old: {self.controller_loop.arm_servo_speed[0]}')
            print(f'value 1: {value}')
            self.controller_loop.arm_servo_speed[0] = value * self.robot_head.arm_control_sensibility
            print(f'servo 1 new: {self.controller_loop.arm_servo_speed[0]}')

    def axis_left_y(self, value: float):
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.controller_loop.speed_x = value * self.robot_head.speed_coefficient
        elif self.robot_head.robot_mode == 'user_control_arm':
            print(f'servo 2 old: {self.controller_loop.arm_servo_speed[1]}')
            print(f'value 2: {value}')
            self.controller_loop.arm_servo_speed[1] = value * self.robot_head.arm_control_sensibility
            print(f'servo 2 new: {self.controller_loop.arm_servo_speed[1]}')

    def axis_right_x(self, value: float):
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.controller_loop.speed_z = (value * self.robot_head.speed_coefficient
                                            * self.robot_head.steer_speed_proportion)
        elif self.robot_head.robot_mode == 'user_control_arm':
            print(f'servo 5 old: {self.controller_loop.arm_servo_speed[4]}')
            print(f'value 5: {value}')
            self.controller_loop.arm_servo_speed[4] = value * self.robot_head.arm_control_sensibility
            print(f'servo 5 new: {self.controller_loop.arm_servo_speed[4]}')

    def axis_right_y(self, value: float):
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_arm':
            print(f'servo 6 old: {self.controller_loop.arm_servo_speed[5]}')
            print(f'value 6: {value}')
            self.controller_loop.arm_servo_speed[5] = value * self.robot_head.arm_control_sensibility
            print(f'servo 6 new: {self.controller_loop.arm_servo_speed[5]}')

    def button_south(self, value: bool):
        # activate buzzer
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.controller_loop.buzzer_is_active = value
        # servo 3 down
        elif self.robot_head.robot_mode == 'user_control_arm':
            print(f'servo 3 old: {self.controller_loop.arm_servo_speed[2]}')
            print(f'value 3: {value}')
            if value:
                self.controller_loop.arm_servo_speed[2] -= self.robot_head.arm_control_sensibility
            else:
                self.controller_loop.arm_servo_speed[2] += self.robot_head.arm_control_sensibility
            print(f'servo 3 new: {self.controller_loop.arm_servo_speed[2]}')

    def button_east(self, value: bool):
        if self.robot_head.robot_mode == 'user_control_wheels':
            if value:
                if self.robot_head.robot_mode == 'user_control_wheels':
                    self.gpio_led.next_color()
        # servo 3 up
        elif self.robot_head.robot_mode == 'user_control_arm':
            print(f'servo 3 old: {self.controller_loop.arm_servo_speed[2]}')
            print(f'value 3: {value}')
            if value:
                self.controller_loop.arm_servo_speed[2] += self.robot_head.arm_control_sensibility
            else:
                self.controller_loop.arm_servo_speed[2] -= self.robot_head.arm_control_sensibility
            print(f'servo 3 new: {self.controller_loop.arm_servo_speed[2]}')

    def button_west(self, value: bool):
        # servo 4 down
        if self.robot_head.robot_mode == 'user_control_arm':
            print(f'servo 4 old: {self.controller_loop.arm_servo_speed[3]}')
            print(f'value 4: {value}')
            if value:
                self.controller_loop.arm_servo_speed[3] -= self.robot_head.arm_control_sensibility
            else:
                self.controller_loop.arm_servo_speed[3] += self.robot_head.arm_control_sensibility
            print(f'servo 4 new: {self.controller_loop.arm_servo_speed[3]}')

    def button_north(self, value: bool):
        # change light effect
        if self.robot_head.robot_mode == 'user_control_wheels':
            if value:
                self.internal_light.next_light_effect()
        # servo 4 up
        elif self.robot_head.robot_mode == 'user_control_arm':
            print(f'servo 4 old: {self.controller_loop.arm_servo_speed[3]}')
            print(f'value 4: {value}')
            if value:
                self.controller_loop.arm_servo_speed[3] += self.robot_head.arm_control_sensibility
            else:
                self.controller_loop.arm_servo_speed[3] -= self.robot_head.arm_control_sensibility
            print(f'servo 4 new: {self.controller_loop.arm_servo_speed[3]}')

    def button_l1(self, value: bool):
        # activate/deactivate hotspot
        if self.robot_head.robot_mode == 'user_control_wheels':
            if value:
                if self.robot_head.hotspot_status == 'inactive':
                    self.robot_head.hotspot_status = 'processing'
                    utils.activate_hotspot(hotspot_ip=self.robot_head.hotspot_ip, verbose=self.verbose)
                    self.robot_head.hotspot_status = 'active'

                elif self.robot_head.hotspot_status == 'active':
                    self.robot_head.hotspot_status = 'processing'
                    utils.deactivate_hotspot(verbose=self.verbose)
                    self.robot_head.hotspot_status = 'inactive'
        # decrease arm servo sensibility
        if self.robot_head.robot_mode == 'user_control_arm':
            if value:
                self.robot_head.decrease_arm_control_sensibility()

    def button_r1(self, value: bool):
        # activate/deactivate ROS2
        if self.robot_head.robot_mode == 'user_control_wheels':
            if value:
                if self.robot_head.ros2_status == 'inactive':
                    self.robot_head.ros2_status = 'processing'
                    utils.activate_ros2(verbose=self.verbose)
                    self.robot_head.ros2_status = 'active'
                elif self.robot_head.ros2_status == 'active':
                    self.robot_head.ros2_status = 'processing'
                    utils.deactivate_ros2(verbose=self.verbose)
                    self.robot_head.ros2_status = 'inactive'
        # increase arm servo sensibility
        if self.robot_head.robot_mode == 'user_control_arm':
            if value:
                self.robot_head.increase_arm_control_sensibility()

    def button_l2(self, value: bool):
        # decrease speed sensibility
        if value:
            self.robot_head.decrease_speed_coefficient()

    def button_r2(self, value: bool):
        # increase speed sensibility
        if value:
            self.robot_head.increase_speed_coefficient()

    def button_select(self, value: bool):
        # switch between user-controlled mode and autonomous mode
        # only allow one press every self.BUTTON_COOLDOWN
        current_time = time.time()
        if value:
            if (current_time - self.last_select_press) >= self.BUTTON_COOLDOWN:
                self.last_select_press = current_time
                self.robot_head.next_mode()
            else:
                if self.verbose >= 2:
                    print('Button SELECT on cooldown...')

    def button_start(self, value: bool):
        current_time = time.time()
        if value:
            if (current_time - self.last_start_press) >= self.BUTTON_COOLDOWN:
                if self.robot_head.robot_mode == 'user_control_arm':
                    self.robot_head.toggle_arm_rigid()
            else:
                if self.verbose >= 2:
                    print('Button START on cooldown...')

    def unknown(self, value):
        if self.verbose >= 2:
            warnings.warn(f'Unknown button with value {value}')
