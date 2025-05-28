#!/usr/bin/env python3
# coding=utf-8
import time
import copy
import warnings

import args
import utils
import global_constants as gc


class ControllerFunctions(object):
    def __init__(self, robot_head, robot_body, arm, **kwargs):

        self.robot_head = robot_head
        self.robot_body = robot_body
        self.arm = arm
        self.internal_light = robot_head.internal_light
        self.led_3_pin = robot_head.led_3_pin
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'controller_interface.yaml', **kwargs)
        self.verbose = parameters['verbose']

        # accept only one button input per cooldown
        # minimum seconds between button presses
        self.button_cooldown = parameters['button_cooldown']
        self.last_button_activation = {}

        # memorize and go-to arm positions
        # start with some predefined positions that can be overwritten
        self.memorized_arm_position = {
            'button_south': copy.deepcopy(self.arm.VERTICAL_POSITION),
            'button_east': copy.deepcopy(self.arm.FOLDED_POSITION),
            'button_west': copy.deepcopy(self.arm.FORWARD_POSITION),
        }

    # value is True or False for buttons
    # value is a float in range [-1, 1] for axes
    # for arrows 'value' is a float in range [-1, 1], but it can only assume the values -1, 0 or 1
    def axis_left_x(self, value: float) -> None:
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            if self.robot_head.robot_sub_mode == gc.SUB_MODE_WHEELS:
                self.robot_head.speed_y = value * self.robot_head.speed_coefficient
            elif self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK:
                # servo 1 (rotate base)
                self.arm.update_speed_fk(servo_id=0, value=-value)
            elif self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK:
                # move gripper left/right
                self.arm.update_speed_ik(value_x=value)

    def axis_left_y(self, value: float) -> None:
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            if self.robot_head.robot_sub_mode == gc.SUB_MODE_WHEELS:
                self.robot_head.speed_x = value * self.robot_head.speed_coefficient
            elif self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK:
                # servo 2
                self.arm.update_speed_fk(servo_id=1, value=-value)
            elif self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK:
                # move gripper forward/backward
                self.arm.update_speed_ik(value_y=value)

    def axis_right_x(self, value: float) -> None:
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            if self.robot_head.robot_sub_mode == gc.SUB_MODE_WHEELS:
                self.robot_head.speed_z = (value * self.robot_head.speed_coefficient
                                           * self.robot_head.steer_speed_proportion)
            elif (self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK
                  or self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK):
                # servo 5 (rotate gripper)
                self.arm.update_speed_fk(servo_id=4, value=value)

    def axis_right_y(self, value: float) -> None:
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            if self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK:
                # move gripper up/down
                # change sign so that pushing the joystick up moves the gripper down and vice versa, which is
                # more intuitive
                self.arm.update_speed_ik(value_z=-value)

    def axis_arrows_x(self, value: float) -> None:
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            if self.robot_head.robot_sub_mode == gc.SUB_MODE_WHEELS:
                self.robot_head.speed_y = value * self.robot_head.speed_coefficient
            elif self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK:
                # servo 4
                self.arm.update_speed_fk(servo_id=3, value=value)
        elif self.robot_head.robot_mode == gc.MODE_AUTONOMOUS_VISION:
            if value > 0:
                self.robot_head.next_target()
            if value < 0:
                self.robot_head.previous_target()

    def axis_arrows_y(self, value: float) -> None:
        assert -1 <= value <= 1, f'Value {value} is out of range [-1, 1]'
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            if self.robot_head.robot_sub_mode == gc.SUB_MODE_WHEELS:
                self.robot_head.speed_x = value * self.robot_head.speed_coefficient
            elif self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK:
                # servo 3
                self.arm.update_speed_fk(servo_id=2, value=-value)
        elif self.robot_head.robot_mode == gc.MODE_AUTONOMOUS_VISION:
            if value > 0:
                self.robot_head.next_model()
            if value < 0:
                self.robot_head.previous_model()

    def button_south(self, value: bool) -> None:
        # memorize current arm position or reach memorized arm position
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            if (self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK
                    or self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK):
                self.memorize_or_set_arm_position(button='button_south', value=value)
        # activate buzzer
            elif self.robot_head.robot_sub_mode == gc.SUB_MODE_WHEELS:
                self.robot_head.buzzer_is_active = value
                self.robot_head.buzzer_state_changed = True
        else:
            self.robot_head.buzzer_is_active = value
            self.robot_head.buzzer_state_changed = True

    def button_east(self, value: bool) -> None:
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            # move robot
            if self.robot_head.robot_sub_mode == gc.SUB_MODE_WHEELS:
                if value:
                    self.led_3_pin.next_color()
            # memorize current arm position or reach memorized arm position
            if (self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK
                    or self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK):
                self.memorize_or_set_arm_position(button='button_east', value=value)

    def button_west(self, value: bool) -> None:
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            if self.robot_head.robot_sub_mode == gc.SUB_MODE_WHEELS:
                if value:
                    if self.cooldown_ended(button='button_west'):
                        self.robot_head.toggle_lidar_listener()
            if (self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK
                    or self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK):
                self.memorize_or_set_arm_position(button='button_west', value=value)

    def button_north(self, value: bool) -> None:
        # memorize current arm position or reach memorized arm position
        # this is the only button without a predefined position already memorized
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            if (self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK
                    or self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK):
                self.memorize_or_set_arm_position(button='button_north', value=value)
        # change internal light effect
            else:
                if value:
                    self.internal_light.next_light_effect()
        else:
            if value:
                self.internal_light.next_light_effect()

    def button_l1(self, value: bool) -> None:
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            # activate/deactivate hotspot
            if self.robot_head.robot_sub_mode == gc.SUB_MODE_WHEELS:
                if value:
                    self.robot_head.toggle_hotspot()
            elif (self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK or
                  self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK):
                # servo 6 open gripper
                self.arm.update_speed_fk(servo_id=5, value=-value)

    def button_r1(self, value: bool) -> None:
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            # activate/deactivate ROS2
            if self.robot_head.robot_sub_mode == gc.SUB_MODE_WHEELS:
                if value:
                    self.robot_head.toggle_ros2_vr_connection()
            elif (self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK or
                  self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK):
                # servo 6 close gripper
                self.arm.update_speed_fk(servo_id=5, value=value)

    def button_l2(self, value: bool) -> None:
        # decrease speed sensibility
        if value:
            self.robot_head.decrease_speed_coefficient()

    def button_r2(self, value: bool) -> None:
        # increase speed sensibility
        if value:
            self.robot_head.increase_speed_coefficient()

    def button_select(self, value: bool) -> None:
        # cycle between robot modes (long press) or sub-modes (short press)
        if value:
            self.start_counting(button='button_select')
        else:
            if self.enough_press_time(button='button_select'):
                self.robot_head.next_mode()
            else:
                self.robot_head.next_sub_mode()

    def button_start(self, value: bool) -> None:
        if value:
            if self.cooldown_ended(button='button_start'):
                self.arm.toggle_rigid()

    def button_rocker_left(self, value: bool) -> None:
        pass

    def button_rocker_right(self, value: bool) -> None:
        pass

    def unknown_input(self, name: str, value) -> None:
        if self.verbose >= 1:
            warnings.warn(f'Unknown button input received (name: {name}, value: {value})')

    def connected(self, controller_id: int) -> None:
        if controller_id not in self.robot_head.controller_id_list:
            self.robot_head.connected_controllers += 1
            self.robot_head.controller_id_list.append(controller_id)
            if self.verbose >= 2:
                print(f'Controller {controller_id} connected')
        else:
            if self.verbose >= 1:
                print(f'Controller with id {controller_id} tried to connect, but this id is already connected')

    def disconnected(self, controller_id: int) -> None:
        if controller_id in self.robot_head.controller_id_list:
            self.robot_head.connected_controllers -= 1
            self.robot_head.controller_id_list.remove(controller_id)
            if self.verbose >= 2:
                print(f'Controller {controller_id} disconnected')
        else:
            if self.verbose >= 1:
                print(f'Controller with id {controller_id} tried to disconnect, but this id is not connected')

    def memorize_or_set_arm_position(self, button: str, value: bool, exclude_gripper_opening: bool = True) -> None:
        # memorize every servo angle of the arm except the gripper opening, because when setting a remembered position
        # the gripper should remain in the current position
        if button not in self.arm.memorizable_button_list:
            self.arm.memorizable_button_list.append(button)
        if value:
            self.led_3_pin.set_color(gc.ORANGE)
            self.start_counting(button=button)
        else:
            self.led_3_pin.set_color(gc.POWER_OFF)
            if self.enough_press_time(button=button):
                self.memorized_arm_position[button] = self.arm.get_safe_arm_angle_list(
                    clamped=True,
                    default_value=90,
                    exclude_gripper_opening=exclude_gripper_opening,
                )
            else:
                if button in self.memorized_arm_position:
                    self.arm.set_desired_angles(angle_list=self.memorized_arm_position[button])

    def cooldown_ended(self, button: str) -> bool:
        # add/check cooldown to button press
        if button not in self.last_button_activation:
            self.last_button_activation[button] = time.time()
            return True

        elapsed_time = time.time() - self.last_button_activation[button]
        if elapsed_time >= self.button_cooldown:
            self.last_button_activation[button] = time.time()
            return True
        else:
            if self.verbose >= 2:
                print(f'Button {button.split("_")[1]} on cooldown...')
            return False

    def start_counting(self, button: str) -> None:
        self.robot_head.one_time_check[button] = True
        self.robot_head.button_press_timestamp[button] = time.time()

    def enough_press_time(self, button: str) -> bool:
        if self.robot_head.button_press_timestamp[button] == 0:
            return False
        # check if button is pressed for enough time
        elapsed_time = time.time() - self.robot_head.button_press_timestamp[button]
        self.robot_head.button_press_timestamp[button] = 0
        if elapsed_time >= self.robot_head.button_press_required_time:
            return True
        else:
            return False

