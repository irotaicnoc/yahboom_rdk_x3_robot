import copy

import args
import utils
import global_constants as gc


class Arm:
    def __init__(self, robot_head, robot_body, **kwargs):
        self.robot_head = robot_head
        self.robot_body = robot_body
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'arm.yaml', **kwargs)
        self.verbose = parameters['verbose']

        arm_initial_angles = robot_body.get_arm_angle_list()
        if arm_initial_angles != [-1, -1, -1, -1, -1, -1]:
            robot_head.robot_sub_mode_dict['user_controlled'].append('arm')
        else:
            raise Exception('The robotic arm is not connected. Mode "user_controlled (arm)" will not be available.')

        self.arm_speed_proportion = parameters['arm_speed_proportion']
        self.is_rigid = True
        self.state_not_updated = True
        # arm servos
        self.servo_speed_list = [0, 0, 0, 0, 0, 0]
        # servo angles have to be in the range [0, 180], except for servo 4 which has range [0, 270]
        # all servos to 90 degrees means vertical position
        # during each loop iteration, the desired angle is updated by adding the speed
        # and the real angle is moved closer to the desired angle
        self.desired_angle_list = arm_initial_angles
        if len(self.desired_angle_list) != 6:
            raise Exception(f'The robot supports a 6-servos arm, the current has {len(self.desired_angle_list)} servos.')
        # speed with which the arm reaches the desired angle [0, 2000]
        # 0 is the fastest speed, 2000 is the slowest speed
        # for manual control use 0, for arbitrary position specified directly via desired_angle_list
        # use a slower speed (higher value)
        self.arm_automated_speed = parameters['arm_automated_speed']
        self.run_time = self.arm_automated_speed[0]
        self.memorizable_button_list = []

    def toggle_rigid(self) -> None:
        self.is_rigid = not self.is_rigid
        self.state_not_updated = True
        if self.verbose >= 2:
            if self.is_rigid:
                print(f'Arm is rigid')
            else:
                print(f'Arm can be moved manually, but cannot be controlled by the controller')

    def set_desired_angles(self, angle_list: list) -> None:
        assert len(angle_list) == len(self.desired_angle_list), \
            (f'Length of angle_list {len(angle_list)} is not equal'
             f' to arm_servos_desired_angle {len(self.desired_angle_list)}')

        self.run_time = utils.change_range(
            value=self.robot_head.speed_coefficient,
            original_min=0.1,
            original_max=1,
            new_min=self.arm_automated_speed[1],
            new_max=self.arm_automated_speed[0],
        )
        self.desired_angle_list = copy.deepcopy(angle_list)

    def update_speed(self, servo_id: int, value) -> None:
        # if the arm was currently performing an automated movement, stop it.
        if self.run_time > 0:
            self.desired_angle_list = self.robot_body.get_arm_angle_list()
        # then apply speed changes due to user input
        self.run_time = 0
        self.servo_speed_list[servo_id] = (value * self.robot_head.speed_coefficient * self.arm_speed_proportion)

    def update_desired_angles(self) -> None:
        for servo_id in range(len(self.servo_speed_list)):
            servo_speed = self.servo_speed_list[servo_id]
            temp_angle = self.desired_angle_list[servo_id] + servo_speed
            if temp_angle < 0:
                temp_angle = 0
            if temp_angle > 180:
                temp_angle = 180
            self.desired_angle_list[servo_id] = temp_angle
