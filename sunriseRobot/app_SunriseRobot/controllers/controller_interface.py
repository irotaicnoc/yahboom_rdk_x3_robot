#!/usr/bin/env python3
# coding=utf-8
import time
import warnings

import args
import utils
import global_constants as gc


class ControllerFunctions(object):
    def __init__(self, robot_head, robot_body, **kwargs):

        self.robot_head = robot_head
        self.robot_body = robot_body
        self.internal_light = robot_head.internal_light
        self.gpio_led = robot_head.gpio_led
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'controller_interface.yaml', **kwargs)
        self.verbose = parameters['verbose']

        # accept only one button input per cooldown
        # minimum seconds between button presses
        self.button_cooldown = parameters['button_cooldown']
        self.last_button_press = {}

        # memorize and go-to arm positions
        self.memorized_arm_position = {}

    # value is True or False for buttons
    # value is a float in range [-1, 1] for axes
    # for arrows 'value' is a float in range [-1, 1], but it can only assume the values -1, 0 or 1
    def axis_left_x(self, value: float) -> None:
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.robot_head.speed_y = value * self.robot_head.speed_coefficient
        elif self.robot_head.robot_mode == 'user_control_arm':
            # servo 1
            self.robot_head.update_arm_speed(servo_id=0, value=value)

    def axis_left_y(self, value: float) -> None:
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.robot_head.speed_x = value * self.robot_head.speed_coefficient
        elif self.robot_head.robot_mode == 'user_control_arm':
            # servo 2
            self.robot_head.update_arm_speed(servo_id=1, value=value)

    def axis_right_x(self, value: float) -> None:
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_wheels':
            self.robot_head.speed_z = (value * self.robot_head.speed_coefficient
                                       * self.robot_head.steer_speed_proportion)
        elif self.robot_head.robot_mode == 'user_control_arm':
            # servo 5
            self.robot_head.update_arm_speed(servo_id=4, value=value)

    def axis_right_y(self, value: float) -> None:
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == 'user_control_arm':
            # servo 6
            self.robot_head.update_arm_speed(servo_id=5, value=value)

    def axis_arrows_x(self, value: float) -> None:
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

    def axis_arrows_y(self, value: float) -> None:
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

    def button_south(self, value: bool) -> None:
        if self.robot_head.robot_mode == 'user_control_arm':
            if value:
                self.robot_head.set_arm_desired_angles(angle_list=[90, 90, 90, 90, 90, 90])
        # activate buzzer
        else:
            self.robot_head.buzzer_is_active = value

    def button_east(self, value: bool) -> None:
        # move robot
        if self.robot_head.robot_mode == 'user_control_wheels':
            if value:
                self.gpio_led.next_color()
        # memorize current arm position or reach memorized arm position
        if self.robot_head.robot_mode == 'user_control_arm':
            self.memorize_or_reach_arm_position(button='button_east', value=value)

    def button_west(self, value: bool) -> None:
        if self.robot_head.robot_mode == 'user_control_wheels':
            if value:
                if self.cooldown(button='button_west'):
                    self.robot_head.toggle_lidar_listener()
        if self.robot_head.robot_mode == 'user_control_arm':
            self.memorize_or_reach_arm_position(button='button_west', value=value)

    def button_north(self, value: bool) -> None:
        # memorize current arm position or reach memorized arm position
        if self.robot_head.robot_mode == 'user_control_arm':
            self.memorize_or_reach_arm_position(button='button_north', value=value)
        # change internal light effect
        else:
            if value:
                self.internal_light.next_light_effect()

    def button_l1(self, value: bool) -> None:
        # activate/deactivate hotspot
        if self.robot_head.robot_mode == 'user_control_wheels':
            if value:
                self.robot_head.toggle_hotspot()

    def button_r1(self, value: bool) -> None:
        # activate/deactivate ROS2
        if self.robot_head.robot_mode == 'user_control_wheels':
            if value:
                self.robot_head.toggle_ros2_vr_connection()

    def button_l2(self, value: bool) -> None:
        # decrease speed sensibility
        if value:
            self.robot_head.decrease_speed_coefficient()

    def button_r2(self, value: bool) -> None:
        # increase speed sensibility
        if value:
            self.robot_head.increase_speed_coefficient()

    def button_select(self, value: bool) -> None:
        # switch between user-controlled mode and autonomous mode
        # only allow one press every self.button_cooldown
        if value:
            if self.cooldown(button='button_select'):
                self.robot_head.next_mode()

    def button_start(self, value: bool) -> None:
        if value:
            if self.cooldown(button='button_start'):
                self.robot_head.toggle_arm_rigid()

    def unknown_input(self, name: str, value) -> None:
        if self.verbose >= 2:
            warnings.warn(f'Unknown button input received (name: {name}, value: {value})')

    def connected(self, controller_id: int) -> None:
        if controller_id not in self.robot_head.controller_id_list:
            self.robot_head.connected_controllers += 1
            self.robot_head.controller_id_list.append(controller_id)
            if self.verbose >= 3:
                print(f'Controller {controller_id} connected')
        else:
            if self.verbose >= 1:
                print(f'Controller with id {controller_id} tried to connect, but this id is already connected')

    def disconnected(self, controller_id: int) -> None:
        if controller_id in self.robot_head.controller_id_list:
            self.robot_head.connected_controllers -= 1
            self.robot_head.controller_id_list.remove(controller_id)
            if self.verbose >= 3:
                print(f'Controller {controller_id} disconnected')
        else:
            if self.verbose >= 1:
                print(f'Controller with id {controller_id} tried to disconnect, but this id is not connected')

    def memorize_or_reach_arm_position(self, button: str, value: bool) -> None:
        if button not in self.robot_head.memorizable_button_list:
            self.robot_head.memorizable_button_list.append(button)
        if self.robot_head.robot_mode == 'user_control_arm':
            if value:
                self.gpio_led.set_color('orange')
                self.robot_head.one_time_check[button] = True
                self.robot_head.button_press_timestamp[button] = time.time()
            else:
                elapsed_time = time.time() - self.robot_head.button_press_timestamp[button]
                self.robot_head.button_press_timestamp[button] = 0
                if elapsed_time >= self.robot_head.button_press_required_time:
                    self.memorized_arm_position[button] = self.robot_body.get_arm_angle_list()
                else:
                    if button in self.memorized_arm_position:
                        self.robot_head.set_arm_desired_angles(angle_list=self.memorized_arm_position[button])
                self.gpio_led.set_color('off')

    def cooldown(self, button: str) -> bool:
        # add/check cooldown to button press
        if button not in self.last_button_press:
            self.last_button_press[button] = time.time()
            return True
        else:
            elapsed_time = time.time() - self.last_button_press[button]
            if elapsed_time >= self.button_cooldown:
                self.last_button_press[button] = time.time()
                return True
            else:
                if self.verbose >= 2:
                    print(f'Button {button.split("_")[1]} on cooldown...')
                return False
