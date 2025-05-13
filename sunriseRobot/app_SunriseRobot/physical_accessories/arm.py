import copy
import numpy as np

import args
import utils
import global_constants as gc


class Arm:
    def __init__(self, robot_head, robot_body, **kwargs):
        self.robot_head = robot_head
        self.robot_body = robot_body
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'arm.yaml', **kwargs)
        self.verbose = parameters['verbose']

        arm_initial_angles = self.get_safe_arm_angle_list(clamped=False)
        if arm_initial_angles == [-1, -1, -1, -1, -1, -1]:
            raise Exception('The robotic arm is not connected. Mode "user_controlled (arm)" will not be available.')

        robot_head.robot_sub_mode_dict['user_controlled'].append('arm')

        # self.arm_speed_proportion = parameters['arm_speed_proportion']
        self.is_rigid = True
        self.state_not_updated = True
        # servo angles have to be in the range [0, 180], except for servo 4 which has range [0, 270]
        # all servos to 90 degrees means vertical position
        self.current_angle_list = arm_initial_angles
        print(f'Arm current angles (initial): {self.current_angle_list}')
        # initial desired and current angles could be in an invalid state because each servo can be rotated by more
        # than 180 degrees by hand. Hence, desired angles must be clamped, and the robot will move the arm from any
        # invalid position to a valid position.
        self.desired_angle_list = self.clamp_angle_list(angle_list=arm_initial_angles)
        print(f'Arm desired angles (initial): {self.desired_angle_list}')

        if len(self.desired_angle_list) != 6:
            raise Exception(f'The robot supports a 6-servo arm, current arm has {len(self.desired_angle_list)} servos')
        # speed with which the arm reaches the desired angle [0, 2000]
        # 0 is the fastest speed, 2000 is the slowest speed
        # for manual control use 0, for arbitrary position specified directly via desired_angle_list
        # use a slower speed (higher value)
        self.run_time = parameters['run_time']
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
        assert len(angle_list) == len(self.desired_angle_list), (f'Length of angle_list {len(angle_list)} is'
             f' not equal to desired_angle_list {len(self.desired_angle_list)}')
        for angle in angle_list:
            assert 0 <= angle <= 180, f'One angle has value {angle}, not in range [0, 180]'
        self.desired_angle_list = copy.deepcopy(angle_list)

    def small_angle_increment(self, servo_id: int, increment: float) -> None:
        # apply speed changes due to user input
        # new_temp_angle = self.current_angle_list[servo_id] + increment * self.arm_speed_proportion
        new_temp_angle = self.current_angle_list[servo_id] + increment
        new_temp_angle = np.clip(new_temp_angle, a_min=0, a_max=180)
        self.desired_angle_list[servo_id] = new_temp_angle

    def small_step_towards_desired_angles(self, max_degree_change: float) -> list:
        # from arm.desired_angle_list calculate the step for each servo.
        # the step is the difference between the desired angle and the current angle, capped to max_degree_change° for
        # each loop iteration
        iteration_angle_step_list = []
        for angle_id in range(len(self.desired_angle_list)):
            iteration_angle_step = self.desired_angle_list[angle_id] - self.current_angle_list[angle_id]
            iteration_angle_step = np.clip(iteration_angle_step, a_min=-max_degree_change, a_max=max_degree_change)
            iteration_angle_step = self.current_angle_list[angle_id] + iteration_angle_step
            iteration_angle_step_list.append(iteration_angle_step)
        return iteration_angle_step_list

    @staticmethod
    def clamp_angle_list(angle_list: list) -> list:
        # clamp angles to [0, 180] for all servos
        clamped_angle_list = []
        for angle in angle_list:
            clamped_angle = np.clip(angle, a_min=0, a_max=180)
            clamped_angle_list.append(clamped_angle)
        return clamped_angle_list

    def get_safe_arm_angle_list(self, clamped: bool = True, retry_limit: int = 5) -> list:
        angle_list = [-1, -1, -1, -1, -1, -1]
        counter = 0
        while -1 in angle_list:
            if counter > 1:
                print(f'first two readings got an error, try n°: {counter}')
                print(f'current angles: {angle_list}')
            temp_angle_list = self.robot_body.get_arm_angle_list()
            for angle_id in range(len(temp_angle_list)):
                angle = temp_angle_list[angle_id]
                if angle != -1:
                    if clamped:
                        # clamp angles to [0, 180] for all servos
                        angle = np.clip(angle, a_min=0, a_max=180)
                    angle_list[angle_id] = angle
            counter += 1
            if counter > retry_limit:
                print(f'Arm angles cannot be read')
                break
        return angle_list
