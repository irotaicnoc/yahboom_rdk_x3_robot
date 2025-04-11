#!/usr/bin/env python3
# coding=utf-8
import time
import warnings

import utils


class ControllerFunctions(object):
    def __init__(self,
                 robot_head,
                 robot_body,
                 internal_light,
                 gpio_led,
                 verbose: int = 0,
                 ):

        self.robot_head = robot_head
        self.robot_body = robot_body
        self.internal_light = internal_light
        self.gpio_led = gpio_led
        self.verbose = verbose

        # accept only one button input per cooldown
        self.last_select_press = 0  # timestamp for SELECT button
        self.last_start_press = 0  # timestamp for START button
        self.BUTTON_COOLDOWN = 5.0  # minimum seconds between button presses

        self.memorized_arm_position = {}

    # value is True or False for buttons
    # value is a float in range [-1, 1] for axes
    # for arrows 'value' is a float in range [-1, 1], but it can only assume the values -1, 0 or 1
    def axis_left_x(self, value: float):
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.robot_head.speed_y = value * self.robot_head.speed_coefficient
        elif self.robot_head.robot_mode == 'user_control_arm':
            # servo 1
            self.robot_head.update_arm_speed(servo_id=0, value=value)

    def axis_left_y(self, value: float):
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.robot_head.speed_x = value * self.robot_head.speed_coefficient
        elif self.robot_head.robot_mode == 'user_control_arm':
            # servo 2
            self.robot_head.update_arm_speed(servo_id=1, value=value)

    def axis_right_x(self, value: float):
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.robot_head.speed_z = (value * self.robot_head.speed_coefficient
                                       * self.robot_head.steer_speed_proportion)
        elif self.robot_head.robot_mode == 'user_control_arm':
            # servo 5
            self.robot_head.update_arm_speed(servo_id=4, value=value)

    def axis_right_y(self, value: float):
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_arm':
            # servo 6
            self.robot_head.update_arm_speed(servo_id=5, value=value)

    def axis_arrows_x(self, value: float):
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.robot_head.speed_y = value * self.robot_head.speed_coefficient
        elif self.robot_head.robot_mode == 'user_control_arm':
            # servo 4
            self.robot_head.update_arm_speed(servo_id=3, value=value)
        elif self.robot_head.robot_mode == 'autonomous_vision':
            if value > 0:
                self.robot_head.next_target()
            if value < 0:
                self.robot_head.previous_target()

    def axis_arrows_y(self, value: float):
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.robot_head.speed_x = value * self.robot_head.speed_coefficient
        elif self.robot_head.robot_mode == 'user_control_arm':
            # servo 3
            self.robot_head.update_arm_speed(servo_id=2, value=value)
        elif self.robot_head.robot_mode == 'autonomous_vision':
            if value > 0:
                self.robot_head.next_model()
            if value < 0:
                self.robot_head.previous_model()

    def button_south(self, value: bool):
        # memorize current arm position or reach memorized arm position
        if self.robot_head.robot_mode == 'user_control_arm':
            if value:
                self.memorize_or_reach_arm_position(button='button_south')
        # activate buzzer
        else:
            self.robot_head.buzzer_is_active = value

    def button_east(self, value: bool):
        # move robot
        if self.robot_head.robot_mode == 'user_control_wheels':
            if value:
                self.gpio_led.next_color()
        # memorize current arm position or reach memorized arm position
        if self.robot_head.robot_mode == 'user_control_arm':
            if value:
                self.memorize_or_reach_arm_position(button='button_east')

    def button_west(self, value: bool):
        # move arm to vertical position
        if self.robot_head.robot_mode == 'user_control_arm':
            if value:
                self.robot_head.set_arm_desired_angles(angle_list=[90, 90, 90, 90, 90, 90])

    def button_north(self, value: bool):
        # memorize current arm position or reach memorized arm position
        if self.robot_head.robot_mode == 'user_control_arm':
            if value:
                self.memorize_or_reach_arm_position(button='button_north')
        # change internal light effect
        else:
            if value:
                self.internal_light.next_light_effect()

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
                    self.last_start_press = current_time
                    self.robot_head.toggle_arm_rigid()
            else:
                if self.verbose >= 2:
                    print('Button START on cooldown...')

    def unknown_input(self, name: str, value):
        if self.verbose >= 2:
            warnings.warn(f'Unknown button input received (name: {name}, value: {value})')

    def connected(self, controller_id: int):
        if controller_id not in self.robot_head.controller_id_list:
            self.robot_head.connected_controllers += 1
            self.robot_head.controller_id_list.append(controller_id)
            if self.verbose >= 3:
                print(f'Controller {controller_id} connected')
        else:
            if self.verbose >= 1:
                print(f'Controller with id {controller_id} tried to connect, but this id is already connected')

    def disconnected(self, controller_id: int):
        if controller_id in self.robot_head.controller_id_list:
            self.robot_head.connected_controllers -= 1
            self.robot_head.controller_id_list.remove(controller_id)
            if self.verbose >= 3:
                print(f'Controller {controller_id} disconnected')
        else:
            if self.verbose >= 1:
                print(f'Controller with id {controller_id} tried to disconnect, but this id is not connected')

    def memorize_or_reach_arm_position(self, button: str):
        print(f'Button {button} pressed')
        if button not in self.memorized_arm_position:
            print('\tnot in dictionary')
            self.memorized_arm_position[button] = None
        if self.memorized_arm_position[button] is None:
            print('\twas empty, memorizing...')
            self.memorized_arm_position[button] = self.robot_body.get_arm_angle_list()
        else:
            if not self.robot_head.arm_is_rigid:
                print('\twas not empty, but arm not rigid, so memorizing...')
                self.memorized_arm_position[button] = self.robot_body.get_arm_angle_list()
            else:
                print('\tarm rigid and not empty, so reaching memorized position...')
                self.robot_head.set_arm_desired_angles(angle_list=self.memorized_arm_position[button])
                print()
