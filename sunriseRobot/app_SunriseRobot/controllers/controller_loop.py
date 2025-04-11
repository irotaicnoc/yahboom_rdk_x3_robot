#!/usr/bin/env python3
# coding=utf-8
import time

import utils


class ControllerLoop(object):
    def __init__(self,
                 robot_body,
                 robot_head,
                 verbose: int = 0,
                 ):

        self.robot_body = robot_body
        self.robot_head = robot_head
        self.verbose = verbose

        # TODO: poi checka se ha senso cambiare le dipendenze fra le classi ControllerLoop,
        #  ControllerFunctions, PS2Controller, e RobotHead

    def update_robot_loop(self):
        assert self.robot_head.connected_controllers >= 0, (f'connected_controllers cannot be negative, but the '
                                                            f'current value is {self.robot_head.connected_controllers}')
        if self.robot_head.connected_controllers == 0:
            if self.verbose >= 2:
                print('No controller connected, waiting for one...')
            time.sleep(2)

        else:
            # buzzer
            if self.robot_head.buzzer_is_active:
                self.robot_body.set_beep(1)
            else:
                self.robot_body.set_beep(0)

            # wheels
            if self.robot_head.robot_mode == 'user_control_wheels':
                self.robot_body.set_car_motion(
                    v_x=self.robot_head.speed_x,
                    v_y=self.robot_head.speed_y,
                    v_z=self.robot_head.speed_z,
                )
            # arm servos
            elif self.robot_head.robot_mode == 'user_control_arm':
                if self.robot_head.arm_state_not_updated:
                    # 20 millisecond beep to signal the change in arm state
                    self.robot_body.set_beep(50)
                    self.robot_head.arm_state_not_updated = False
                    # manually set configuration is maintained
                    if self.robot_head.arm_is_rigid:
                        self.robot_head.arm_servos_desired_angle = self.robot_body.get_uart_servo_angle_array()
                    self.robot_body.set_uart_servo_torque(enable=self.robot_head.arm_is_rigid)

                if self.robot_head.arm_is_rigid:
                    self.robot_head.update_servos_desired_angle()
                    self.robot_body.set_uart_servo_angle_array(
                        angle_s=self.robot_head.arm_servos_desired_angle,
                        run_time=0,
                    )
            else:
                time.sleep(2)

            time.sleep(0.02)
