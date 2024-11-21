import args


class RobotHead:
    def __init__(self, **kwargs):
        parameters = args.import_args(
            yaml_path='/root/sunriseRobot/app_SunriseRobot/configs/robot_head_config.yaml',
            **kwargs,
        )
        # autonomous mode parameters
        self.robot_mode_list = ['user_controlled', ]
        self.robot_mode = self.robot_mode_list[0]
        self.tracking_target_list = ['person', 'cat', 'backpack', 'fork', 'spoon',
                                     'orange', 'chair', 'remote', 'cell phone']
        self.tracking_target_pos = 0

        self.speed_coefficient = parameters['speed_coefficient']
        self.verbose = parameters['verbose']

    def next_mode(self):
        if self.verbose:
            print(f"Switching from {self.robot_mode} mode.")
        self.robot_mode = self.robot_mode_list[(self.robot_mode_list.index(self.robot_mode) + 1) % len(self.robot_mode_list)]
        if self.verbose:
            print(f"Switching to {self.robot_mode} mode.")

    def next_target(self):
        self.tracking_target_pos += 1
        self.tracking_target_pos = self.tracking_target_pos % len(self.tracking_target_list)
        if self.verbose:
            print(f'New target: {self.tracking_target_list[self.tracking_target_pos]}')

    def previous_target(self):
        self.tracking_target_pos -= 1
        self.tracking_target_pos = self.tracking_target_pos % len(self.tracking_target_list)
        if self.verbose:
            print(f'New target: {self.tracking_target_list[self.tracking_target_pos]}')
