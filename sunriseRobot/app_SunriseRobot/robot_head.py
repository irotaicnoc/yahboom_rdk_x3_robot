import args
import smbus

import global_constants as gc


class RobotHead:
    def __init__(self, **kwargs):
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'robot_head_config.yaml',
            **kwargs,
        )
        # autonomous mode parameters
        self.robot_mode_list = ['user_controlled', ]
        self.robot_mode = self.robot_mode_list[0]
        self.tracking_target_list = parameters['tracking_target_list']
        self.tracking_target_pos = 0

        # fan and lights parameters
        self.bus = smbus.SMBus(0)
        self.fan_state = gc.FAN_START_CMD
        self.light_state = gc.LIGHT_STOP_CMD

        # hotspot and ROS2 parameters
        self.hotspot_status = 'inactive'
        self.hotspot_ip = parameters['hotspot_ip']
        self.ros2_status = 'inactive'

        # motion parameters
        self.steer_speed_proportion = parameters['steer_speed_proportion']
        self.speed_coefficient = parameters['speed_coefficient']
        self.verbose = parameters['verbose']

    def next_mode(self):
        if self.verbose >= 2:
            print(f'Switching from {self.robot_mode} mode.')
        self.robot_mode = self.robot_mode_list[
            (self.robot_mode_list.index(self.robot_mode) + 1) % len(self.robot_mode_list)
        ]
        if self.verbose >= 2:
            print(f'Switching to {self.robot_mode} mode.')

    def next_target(self):
        self.tracking_target_pos += 1
        self.tracking_target_pos = self.tracking_target_pos % len(self.tracking_target_list)
        if self.verbose >= 1:
            print(f'New target: {self.tracking_target_list[self.tracking_target_pos]}')

    def previous_target(self):
        self.tracking_target_pos -= 1
        self.tracking_target_pos = self.tracking_target_pos % len(self.tracking_target_list)
        if self.verbose >= 1:
            print(f'New target: {self.tracking_target_list[self.tracking_target_pos]}')

    def increase_speed_coefficient(self):
        self.speed_coefficient = min(1.0, self.speed_coefficient + 0.1)

    def decrease_speed_coefficient(self):
        self.speed_coefficient = max(0.0, self.speed_coefficient - 0.1)

    def next_light_effect(self):
        if self.verbose >= 2:
            print(f'Switching from {self.light_state} mode.')
        self.light_state = gc.LIGHT_EFFECT_CMD_LIST[
            (gc.LIGHT_EFFECT_CMD_LIST.index(self.light_state) + 1) % len(gc.LIGHT_EFFECT_CMD_LIST)
        ]
        if self.verbose >= 2:
            print(f'Switching to {self.light_state} mode.')

    def turn_off_light(self):
        self.light_state = gc.LIGHT_STOP_CMD

