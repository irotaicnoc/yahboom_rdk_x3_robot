#!/usr/bin/env python3
# coding=utf-8
import time
import warnings

import utils


class ControllerLoop(object):
    def __init__(self,
                 robot_body,
                 robot_head,
                 arm_servos_initial_angles: list = [90, 90, 90, 90, 90, 90],
                 verbose: int = 0,
                 ):

        self.robot_body = robot_body
        self.robot_head = robot_head
        self.verbose = verbose
        self.connected_controllers = 0

        # TODO: sposta queste variabili in robot_head perchè possono essere usate anche da agenti autonomi
        # TODO: poi checka se ha senso cambiare le dipendenze fra le classi ControllerLoop,
        #  ControllerFunctions, PS2Controller, e RobotHead
        # wheel speed
        self.speed_x = 0
        self.speed_y = 0
        self.speed_z = 0
        # buzzer
        self.buzzer_is_active = False
        # arm servos
        self.arm_servo_speed = [0, 0, 0, 0, 0, 0]
        # servo angles have to be in the range [0, 180], except for servo 4 which has range [0, 270]
        # all servos to 90 degrees means vertical position
        # during each loop iteration, the desired angle is updated by adding the speed
        # and the real angle is moved closer to the desired angle
        self.arm_servos_desired_angle = arm_servos_initial_angles
        if len(self.arm_servos_desired_angle) > 6:
            warnings.warn(f'controls supports at most a 6-servos arm,'
                          f' but {len(self.arm_servos_desired_angle)} were provided. ')

    def update_servos_desired_angle(self) -> None:
        for servo_id in range(len(self.arm_servo_speed)):
            servo_speed = self.arm_servo_speed[servo_id]
            temp_angle = self.arm_servos_desired_angle[servo_id] + servo_speed
            if temp_angle < 0:
                temp_angle = 0
            if temp_angle > 180:
                temp_angle = 180
            self.arm_servos_desired_angle[servo_id] = temp_angle

    def update_robot_loop(self):
        assert self.connected_controllers >= 0, (f'connected_controllers cannot be negative,'
                                                 f' but the current value is {self.connected_controllers}')
        if self.connected_controllers == 0:
            if self.verbose >= 2:
                print('No controller connected, waiting for one...')
            time.sleep(2)

        else:
            # buzzer
            if self.buzzer_is_active:
                self.robot_body.set_beep(1)
            else:
                self.robot_body.set_beep(0)

            # wheels
            if self.robot_head.robot_mode == 'user_control_wheels':
                self.robot_body.set_car_motion(self.speed_x, self.speed_y, self.speed_z)
            # arm servos
            elif self.robot_head.robot_mode == 'user_control_arm':
                self.update_servos_desired_angle()
                self.robot_body.set_uart_servo_angle_array(angle_s=self.arm_servos_desired_angle, run_time=0)
            else:
                time.sleep(2)

            time.sleep(0.02)
