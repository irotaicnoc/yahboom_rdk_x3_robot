#!/usr/bin/env python3
# coding=utf-8
import os
import time
import copy
import numpy as np

import args
import utils
import global_constants as gc
from physical_accessories.lidar_listener import ThreadedLidarListener


class ControllerLoop(object):
    def __init__(self,
                 robot_body,
                 robot_head,
                 verbose: int = 0,
                 ):
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'controller_loop.yaml', verbose=verbose)
        self.robot_body = robot_body
        self.robot_head = robot_head
        self.gpio_led = robot_head.gpio_led
        self.beep_time = parameters['beep_time']
        self.verbose = parameters['verbose']

        # lidar initialization
        self.lidar_listener = None
        self.lidar_is_active = False
        self.lidar_kwargs = parameters['lidar_kwargs']
        self.min_allowed_distance = parameters['min_allowed_distance']
        # print lidar and direction to console
        self.circle_radius = 15
        self.circle_diameter = self.circle_radius * 2
        self.dist_proportion = self.circle_radius / self.lidar_kwargs['response_dist']
        self.base_canvas = [[' ' for _ in range(self.circle_diameter)] for _ in range(self.circle_diameter)]
        #   add detection area (circle)
        for i in range(self.circle_diameter):
            for j in range(self.circle_diameter):
                if (i - self.circle_radius) ** 2 + (j - self.circle_radius) ** 2 <= self.circle_radius ** 2:
                    self.base_canvas[i][j] = '.'
                    if j == 0:
                        self.base_canvas[i].append('.')
        self.base_canvas.append(copy.deepcopy(self.base_canvas[0]))
        #   add the robot
        self.base_canvas[self.circle_radius - 1][self.circle_radius - 1] = '/'
        self.base_canvas[self.circle_radius - 1][self.circle_radius + 1] = '\\'
        self.base_canvas[self.circle_radius + 1][self.circle_radius - 1] = '\\'
        self.base_canvas[self.circle_radius + 1][self.circle_radius + 1] = '/'
        self.base_canvas[self.circle_radius][self.circle_radius - 1] = '|'
        self.base_canvas[self.circle_radius][self.circle_radius + 1] = '|'
        self.base_canvas[self.circle_radius][self.circle_radius] = 'R'
        self.base_canvas[self.circle_radius - 1][self.circle_radius] = '^'
        self.base_canvas[self.circle_radius + 1][self.circle_radius] = '_'

    def update_robot_loop(self) -> None:
        assert self.robot_head.connected_controllers >= 0, (f'connected_controllers cannot be negative, but the '
                                                            f'current value is {self.robot_head.connected_controllers}')
        if self.robot_head.connected_controllers == 0:
            if self.verbose >= 2:
                print('No controller connected, waiting for one...')
            time.sleep(2)
            return

        # buzzer
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
        if self.robot_head.robot_mode == 'user_control_wheels':
            if self.lidar_is_active:
                # get the lidar data
                obstacles_by_sector, average_distance_by_sector = self.lidar_listener.get_obstacles_by_sector()
                if obstacles_by_sector is not None:
                    self.print_state_ascii(
                        obstacles_by_sector=obstacles_by_sector,
                        average_distance_by_sector=average_distance_by_sector,
                    )
                    # calculate the direction of the robot given speed_x, speed_y, speed_z

                    # # check if there are obstacles in the direction of the robot
                    # if utils.check_obstacles_in_direction(
                    #         obstacles_by_sector=obstacles_by_sector,
                    #         average_distance_by_sector=average_distance_by_sector,
                    #         robot_direction=robot_direction,
                    #         min_allowed_distance=self.min_allowed_distance,
                    # ):
                    #     # stop the robot if there are obstacles in the direction of the robot
                    #     self.robot_body.set_car_motion(v_x=0, v_y=0, v_z=0)
                    #     if self.verbose >= 2:
                    #         print('Obstacle detected, stopping the robot.')
                    #     time.sleep(0.5)
                    #     return

            self.robot_body.set_car_motion(
                v_x=self.robot_head.speed_x,
                v_y=self.robot_head.speed_y,
                v_z=self.robot_head.speed_z,
            )
        # arm servos
        elif self.robot_head.robot_mode == 'user_control_arm':
            if self.robot_head.arm_state_not_updated:
                # beep to signal the change in arm state
                self.robot_body.set_beep(self.beep_time)
                self.robot_head.arm_state_not_updated = False
                # manually set configuration is maintained
                if self.robot_head.arm_is_rigid:
                    self.robot_head.arm_desired_angles = self.robot_body.get_arm_angle_list()
                self.robot_body.set_arm_torque(enable=self.robot_head.arm_is_rigid)

            if self.robot_head.button_press_time != 0:
                elapsed_time = time.time() - self.robot_head.button_press_time
                if elapsed_time >= 2:
                    if self.robot_head.one_time_check:
                        self.robot_head.gpio_led.set_color('green')
                        self.robot_head.one_time_check = False
                        self.robot_body.set_beep(self.beep_time)

            if self.robot_head.arm_is_rigid:
                self.robot_head.update_arm_desired_angles()
                self.robot_body.set_arm_angle_list(
                    angle_s=self.robot_head.arm_desired_angles,
                    run_time=self.robot_head.run_time,
                )
        else:
            time.sleep(2)

        time.sleep(0.02)

    def print_state_ascii(self, obstacles_by_sector, average_distance_by_sector):
        os.system('clear')
        canvas = copy.deepcopy(self.base_canvas)

        # add detected obstacles using their direction and distance
        if obstacles_by_sector is not None:
            for sector_num in range(len(obstacles_by_sector)):
                if obstacles_by_sector[sector_num]:
                    angle_grad = sector_num * self.lidar_listener.sector_angle
                    angle_grad = (angle_grad + 90) % 360
                    angle_rad = np.deg2rad(angle_grad)
                    distance = average_distance_by_sector[sector_num]
                    x = int(self.circle_radius + distance * self.dist_proportion * np.cos(angle_rad))
                    y = int(self.circle_radius - distance * self.dist_proportion * np.sin(angle_rad))
                    if 0 <= x < self.circle_diameter and 0 <= y < self.circle_diameter:
                        canvas[y][x] = '#'

        robot_direction = utils.calculate_robot_direction(
            speed_x=self.robot_head.speed_x,
            speed_y=self.robot_head.speed_y,
            speed_z=self.robot_head.speed_z,
        )
        for i in range(2, 6):
            x = int(self.circle_radius + i * np.cos(np.deg2rad(robot_direction)))
            y = int(self.circle_radius - i * np.sin(np.deg2rad(robot_direction)))
            if 0 <= x < self.circle_diameter and 0 <= y < self.circle_diameter:
                canvas[y][x] = 'o'

        for row in canvas:
            print(' '.join(row))
        time.sleep(0.1)

    def start_lidar_listener(self):
        if self.lidar_listener is None:
            try:
                self.lidar_listener = ThreadedLidarListener(**self.lidar_kwargs, verbose=self.verbose)
                self.lidar_is_active = True
                self.robot_head.lidar_listener_status = 'active'
                if self.verbose >= 2:
                    print('Lidar listener started')
            except Exception as e:
                print('Failed to start lidar listener for user motion safeguard with error')
                print(e)
                print(e.__traceback__)
                self.lidar_is_active = False
                self.lidar_listener = None
                self.robot_head.lidar_listener_status = 'inactive'
        else:
            self.lidar_is_active = True
            self.robot_head.lidar_listener_status = 'active'
            if self.verbose >= 2:
                print('Lidar listener already started')

    def stop_lidar_listener(self):
        if self.lidar_listener is not None:
            self.lidar_listener.delete_listener()
            self.lidar_listener = None
            if self.verbose >= 2:
                print('Lidar listener stopped')
        else:
            if self.verbose >= 2:
                print('Lidar listener already stopped')
        self.lidar_is_active = False
        self.robot_head.lidar_listener_status = 'inactive'
