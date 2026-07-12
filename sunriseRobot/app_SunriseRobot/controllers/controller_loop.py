#!/usr/bin/env python3
# coding=utf-8
# import os
import time
# import copy
# import numpy as np

import args
import utils
import global_constants as gc
from ros2.lidar_listener import ThreadedLidarListener


class ControllerLoop(object):
    def __init__(self, robot_body, robot_head, arm, verbose: int = 0):
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'controller_loop.yaml', verbose=verbose)
        self.robot_body = robot_body
        self.robot_head = robot_head
        self.arm = arm
        self.led_3_pin = robot_head.led_3_pin
        self.loop_sleep_time = parameters['loop_sleep_time']
        self.verbose = parameters['verbose']

        # lidar initialization
        self.lidar_listener = None
        self.lidar_is_active = False
        self.lidar_parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'lidar_listener.yaml',
            verbose=verbose,
        )
        self.min_allowed_distance = parameters['min_allowed_distance']
        assert self.min_allowed_distance[0] < self.min_allowed_distance[1], (f'min_allowed_distance[0] ('
                                                        f'{self.min_allowed_distance[0]}) must be smaller than '
                                                        f'min_allowed_distance[1] ({self.min_allowed_distance[1]}).')
        self.obstacles_by_sector = None
        self.average_distance_by_sector = None
        # print lidar and direction to console
        # self.circle_radius = 15
        # self.circle_diameter = self.circle_radius * 2
        # self.dist_proportion = self.circle_radius / self.lidar_parameters['response_dist']
        # self.base_canvas = [[' ' for _ in range(self.circle_diameter)] for _ in range(self.circle_diameter)]
        # #   add detection area (circle)
        # for i in range(self.circle_diameter):
        #     for j in range(self.circle_diameter):
        #         if (i - self.circle_radius) ** 2 + (j - self.circle_radius) ** 2 <= self.circle_radius ** 2:
        #             self.base_canvas[i][j] = '.'
        #             if j == 0:
        #                 self.base_canvas[i].append('.')
        # self.base_canvas.append(copy.deepcopy(self.base_canvas[0]))
        # #   add the robot
        # self.base_canvas[self.circle_radius - 1][self.circle_radius - 1] = '/'
        # self.base_canvas[self.circle_radius - 1][self.circle_radius + 1] = '\\'
        # self.base_canvas[self.circle_radius + 1][self.circle_radius - 1] = '\\'
        # self.base_canvas[self.circle_radius + 1][self.circle_radius + 1] = '/'
        # self.base_canvas[self.circle_radius][self.circle_radius - 1] = '|'
        # self.base_canvas[self.circle_radius][self.circle_radius + 1] = '|'
        # self.base_canvas[self.circle_radius][self.circle_radius] = 'R'
        # self.base_canvas[self.circle_radius - 1][self.circle_radius] = '^'
        # self.base_canvas[self.circle_radius + 1][self.circle_radius] = '_'

    def update_robot_loop(self) -> None:
        assert self.robot_head.connected_controllers >= 0, (f'connected_controllers cannot be negative, but the '
                                                            f'current value is {self.robot_head.connected_controllers}')
        if self.robot_head.connected_controllers == 0:
            if self.verbose >= 2:
                print('No controller connected, waiting for one...')
            time.sleep(2)
            return

        # buzzer
        # send command to the buzzer only if the state has changed from the previous loop iteration
        if self.robot_head.buzzer_state_changed:
            self.robot_head.buzzer_state_changed = False
            if self.robot_head.buzzer_is_active:
                self.robot_body.set_beep(1)
            else:
                self.robot_body.set_beep(0)

        # activate/deactivate lidar listener
        if self.robot_head.lidar_listener_status == 'processing':
            if not self.lidar_is_active:
                self.start_lidar_listener()
            else:
                self.stop_lidar_listener()

        # wheels
        if self.robot_head.robot_mode == gc.MODE_USER_CONTROLLED:
            # arm buttons
            if (self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK
                    or self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK):
                for button in self.robot_head.button_press_timestamp:
                    timestamp = self.robot_head.button_press_timestamp[button]
                    if timestamp != 0:
                        if button in self.arm.memorizable_button_list:
                            if time.time() - timestamp >= self.robot_head.button_press_required_time:
                                if self.robot_head.one_time_check[button]:
                                    self.robot_head.led_3_pin.set_color(gc.GREEN)
                                    self.robot_head.one_time_check[button] = False
                                    self.robot_body.set_beep(gc.SHORT_BEEP)

            if self.lidar_is_active:
                # get the lidar data
                obstacles_by_sector, average_distance_by_sector = self.lidar_listener.get_obstacles_by_sector()
                if obstacles_by_sector is not None:
                    self.obstacles_by_sector = obstacles_by_sector
                    self.average_distance_by_sector = average_distance_by_sector
                if self.obstacles_by_sector is not None:
                    robot_direction = utils.calculate_robot_direction(
                        speed_x=self.robot_head.speed_x,
                        speed_y=self.robot_head.speed_y,
                        speed_z=self.robot_head.speed_z / self.robot_head.speed_coefficient,
                    )
                    if robot_direction is not None:
                        # self.print_state_ascii(
                        #     obstacles_by_sector=self.obstacles_by_sector,
                        #     average_distance_by_sector=self.average_distance_by_sector,
                        #     robot_direction=robot_direction,
                        # )
                        # calculate the direction of the robot given speed_x, speed_y, speed_z
                        # robot_direction is an angle in degrees in range [0, 360)
                        sector_num = int(((robot_direction - 90) % 360) / self.lidar_listener.sector_angle)
                        preceding_sector_num = (sector_num - 1) % len(self.obstacles_by_sector)
                        following_sector_num = (sector_num + 1) % len(self.obstacles_by_sector)
                        obstacle = False
                        min_allowed_distance = utils.change_range(
                            value=self.robot_head.speed_coefficient,
                            original_min=0.1,
                            original_max=1.0,
                            new_min=self.min_allowed_distance[0],
                            new_max=self.min_allowed_distance[1],
                        )
                        if (self.obstacles_by_sector[sector_num] and
                                self.average_distance_by_sector[sector_num] < min_allowed_distance):
                            obstacle = True
                        if (self.obstacles_by_sector[preceding_sector_num] and
                                self.average_distance_by_sector[preceding_sector_num] < min_allowed_distance):
                            obstacle = True
                        if (self.obstacles_by_sector[following_sector_num] and
                                self.average_distance_by_sector[following_sector_num] < min_allowed_distance):
                            obstacle = True
                        if obstacle:
                            self.robot_body.set_beep(gc.SHORT_BEEP)
                            # allow only rotation
                            self.robot_body.set_car_motion(v_x=0, v_y=0, v_z=self.robot_head.speed_z)
                            if self.verbose >= 2:
                                print('obstacle detected, stopping the robot')
                            time.sleep(0.1)
                            return

            self.robot_head.check_programmed_stop()
            self.robot_body.set_car_motion(
                v_x=self.robot_head.speed_x,
                v_y=self.robot_head.speed_y,
                v_z=self.robot_head.speed_z,
            )

            # Only drive the arm while an arm sub-mode is active. In wheels sub-mode the arm is already
            # folded and held rigid by sub_mode_wheel_start_callback, so re-sending its angles every loop is
            # redundant and just puts arm frames on the shared UART for no reason.
            if self.arm.is_rigid and self.robot_head.robot_sub_mode != gc.SUB_MODE_WHEELS:
                self.arm.update_desired_angles()
                self.robot_body.set_arm_angle_list(angle_s=self.arm.desired_angle_list, run_time=self.arm.run_time)

        else:
            time.sleep(2)

        time.sleep(self.loop_sleep_time)

    # def print_state_ascii(self,
    #                       obstacles_by_sector: list,
    #                       average_distance_by_sector: list,
    #                       robot_direction: float
    #                       ) -> None:
    #     os.system('clear')
    #     canvas = copy.deepcopy(self.base_canvas)
    #
    #     # add detected obstacles using their direction and distance
    #     if obstacles_by_sector is not None:
    #         for sector_number in range(len(obstacles_by_sector)):
    #             if obstacles_by_sector[sector_number]:
    #                 angle_degrees = utils.circular_sector_to_degree_angle(
    #                     sector_number=sector_number,
    #                     sector_angle=self.lidar_listener.sector_angle,
    #                 )
    #                 angle_radian = np.deg2rad(angle_degrees)
    #                 distance = average_distance_by_sector[sector_number]
    #                 x = int(self.circle_radius + distance * self.dist_proportion * np.cos(angle_radian))
    #                 y = int(self.circle_radius - distance * self.dist_proportion * np.sin(angle_radian))
    #                 if 0 <= x < self.circle_diameter and 0 <= y < self.circle_diameter:
    #                     canvas[y][x] = '#'
    #
    #     for i in range(2, 6):
    #         x = int(self.circle_radius + i * np.cos(np.deg2rad(robot_direction)))
    #         y = int(self.circle_radius - i * np.sin(np.deg2rad(robot_direction)))
    #         if 0 <= x < self.circle_diameter and 0 <= y < self.circle_diameter:
    #             canvas[y][x] = 'o'
    #
    #     for row in canvas:
    #         print(' '.join(row))
    #     time.sleep(0.1)

    def start_lidar_listener(self) -> None:
        if self.lidar_listener is None:
            try:
                self.lidar_listener = ThreadedLidarListener(**self.lidar_parameters)
                self.lidar_is_active = True
                self.robot_head.lidar_listener_status = 'active'
                if self.verbose >= 2:
                    print('Lidar listener started')
            except Exception as e:
                utils.print_exception(
                    exception=e,
                    message='Failed to start lidar listener for user motion safeguard with error',
                )
                self.lidar_is_active = False
                self.lidar_listener = None
                self.robot_head.lidar_listener_status = 'inactive'
        else:
            self.lidar_is_active = True
            self.robot_head.lidar_listener_status = 'active'
            if self.verbose >= 2:
                print('Lidar listener already started')

    def stop_lidar_listener(self) -> None:
        if self.lidar_listener is not None:
            self.lidar_listener.delete_listener()
            self.lidar_listener = None
        else:
            if self.verbose >= 1:
                print('Lidar listener already stopped')
        self.lidar_is_active = False
        self.robot_head.lidar_listener_status = 'inactive'
