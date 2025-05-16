import copy
import time
import warnings
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
        if arm_initial_angles != [-1, -1, -1, -1, -1, -1]:
            robot_head.robot_sub_mode_dict[gc.MODE_USER_CONTROLLED].append(gc.SUB_MODE_ARM_FK)
            robot_head.sub_mode_change_callbacks[gc.SUB_MODE_ARM_FK] = self.sub_mode_fk_start_callback
        else:
            raise Exception('The robotic arm is not connected. Mode "user_controlled (arm_fk)" will not be available')

        self.arm_speed_proportion_fk = parameters['arm_speed_proportion_fk']
        self.is_rigid = True
        self.state_not_updated = True
        # arm servos
        self.servo_speed_list = [0, 0, 0, 0, 0, 0]
        # servo angles have to be in the range [0, 180], except for servo 4 which has range [0, 270]
        # all servos to 90 degrees means vertical position
        # during each loop iteration, the desired angle is updated by adding the speed
        # and the real angle is moved closer to the desired angle
        self.desired_angle_list = self.clamp_angle_list(arm_initial_angles, default_value=90)
        if len(self.desired_angle_list) != 6:
            raise Exception(f'The robot supports a 6-servo arm, current arm has {len(self.desired_angle_list)} servos.')
        # speed with which the arm reaches the desired angle [0, 2000]
        # 0 is the fastest speed, 2000 is the slowest speed
        # for manual control use 0, for arbitrary position specified directly via desired_angle_list
        # use a slower speed (higher value)
        self.arm_automated_speed = parameters['arm_automated_speed']
        self.run_time = self.arm_automated_speed[0]
        self.memorizable_button_list = []

        try:
            from ikpy.chain import Chain
        except ImportError:
            raise ImportError('The ikpy library for inverse kinematics is not installed.'
                              ' Mode "user_controlled (arm_ik)" will not be available')

        self.arm_speed_proportion_ik = parameters['arm_speed_proportion_ik']
        self.servo_chain = Chain.from_urdf_file(gc.URDF_FOLDER_PATH + 'arm.urdf')
        robot_head.robot_sub_mode_dict[gc.MODE_USER_CONTROLLED].append(gc.SUB_MODE_ARM_IK)
        robot_head.sub_mode_change_callbacks[gc.SUB_MODE_ARM_IK] = self.sub_mode_ik_start_callback
        # these are the coordinates of the gripper in the robot's coordinate system. In inverse kinematics mode
        # they are used in place of the desired angles for motors 0, 1, 2, 3. Only motors 4 (gripper rotation)
        # and 5 (gripper opening) are controlled in the same way in both sub modes.
        self.gripper_pos = [0, 0, 0]
        # # speed of the gripper in the x, y, z directions
        self.gripper_speed = [0, 0, 0]
        # initializes gripper position and speed, and desired angles
        self.sub_mode_ik_start_callback()

    def toggle_rigid(self) -> None:
        self.is_rigid = not self.is_rigid
        self.state_not_updated = True
        if self.verbose >= 2:
            if self.is_rigid:
                print(f'Arm is rigid')
            else:
                print(f'Arm can be moved manually, but cannot be controlled by the controller')

    def set_desired_angles(self, angle_list: list) -> None:
        assert len(angle_list) == len(self.desired_angle_list), (f'Length of angle_list {len(angle_list)} is not '
            f'equal to arm_servos_desired_angle {len(self.desired_angle_list)}')

        self.run_time = utils.change_range(
            value=self.robot_head.speed_coefficient,
            original_min=0.1,
            original_max=1,
            new_min=self.arm_automated_speed[1],
            new_max=self.arm_automated_speed[0],
        )
        self.desired_angle_list = copy.deepcopy(angle_list)

    def update_speed_fk(self, servo_id: int, value) -> None:
        # This function directly modifies the speed of the servo with id servo_id
        # if the arm was currently performing an automated movement, stop it.
        if self.run_time > 0:
            self.desired_angle_list = self.get_safe_arm_angle_list(clamped=True, default_value=90)
            self.run_time = 0
        # then apply speed changes due to user input
        self.servo_speed_list[servo_id] = (value * self.robot_head.speed_coefficient * self.arm_speed_proportion_fk)

    def update_speed_ik(self, value_x: float = None, value_y: float = None, value_z: float = None) -> None:
        # This function modifies the speed of the gripper in the x, y, z directions
        # if the arm was currently performing an automated movement, stop it.
        if self.run_time > 0:
            self.desired_angle_list = self.get_safe_arm_angle_list(clamped=True, default_value=90)
            self.run_time = 0
        # then apply speed changes due to user input
        if value_x is not None:
            self.gripper_speed[0] = value_x * self.robot_head.speed_coefficient * self.arm_speed_proportion_ik
            # print(f'gripper_speed x: {self.gripper_speed[0]}')
        if value_y is not None:
            self.gripper_speed[1] = value_y * self.robot_head.speed_coefficient * self.arm_speed_proportion_ik
            # print(f'gripper_speed y: {self.gripper_speed[1]}')
        if value_z is not None:
            self.gripper_speed[2] = value_z * self.robot_head.speed_coefficient * self.arm_speed_proportion_ik
            # print(f'gripper_speed z: {self.gripper_speed[2]}')

    def update_desired_angles(self) -> None:
        if self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_FK:
            for servo_id in range(len(self.servo_speed_list)):
                servo_speed = self.servo_speed_list[servo_id]
                temp_angle = self.desired_angle_list[servo_id] + servo_speed
                self.desired_angle_list[servo_id] = np.clip(temp_angle, a_min=0, a_max=180)
        elif self.robot_head.robot_sub_mode == gc.SUB_MODE_ARM_IK:
            # update the gripper position in the robot's coordinate system
            # the gripper position is used in place of the desired angles for motors 0, 1, 2, 3
            # print(f'gripper_pos before: {self.gripper_pos}')
            self.gripper_pos[0] += self.gripper_speed[0]
            self.gripper_pos[1] += self.gripper_speed[1]
            self.gripper_pos[2] += self.gripper_speed[2]
            # print(f'gripper_pos after:  {self.gripper_pos}')
            # the function returns 6 angle_list, but we don't need the first and last ones, they should be the
            # gripper rotation and opening. But the 2 excluded angles are the last 2 angles in the list
            start_time = time.time()
            ikpy_angle_list = self.servo_chain.inverse_kinematics(target_position=self.gripper_pos)
            print(f'IK computation time: {round(time.time() - start_time, 2)} seconds')
            # print(f'ikpy_angle_list: {ikpy_angle_list}')
            # print(f'desired_angle_list before: {self.desired_angle_list}')
            self.desired_angle_list[:4] = self.ikpy_to_degree_conversion(ikpy_angle_list)
            # print(f'desired_angle_list after: {self.desired_angle_list}')
            # the last two angles (4 and 5) are updated normally
            temp_angle_4 = self.desired_angle_list[4] + self.servo_speed_list[4]
            temp_angle_5 = self.desired_angle_list[5] + self.servo_speed_list[5]
            self.desired_angle_list[4] = np.clip(temp_angle_4, a_min=0, a_max=180)
            self.desired_angle_list[5] = np.clip(temp_angle_5, a_min=0, a_max=180)

    @staticmethod
    def clamp_angle_list(angle_list: list, default_value: int = -1) -> list:
        # clamp angles to [0, 180] for all servos
        clamped_angle_list = []
        for angle in angle_list:
            if default_value != -1 and angle == -1:
                clamped_angle = default_value
            else:
                clamped_angle = np.clip(angle, a_min=0, a_max=180)
            clamped_angle_list.append(clamped_angle)
        return clamped_angle_list

    def get_safe_arm_angle_list(self, clamped: bool = True, retry_limit: int = 10, default_value: int = -1) -> list:
        angle_list = [-1, -1, -1, -1, -1, -1]
        counter = 0
        while -1 in angle_list:
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
                if self.verbose >= 1:
                    warnings.warn(f'Arm angles cannot be read, angles: {angle_list}')
                if default_value != -1:
                    for angle_id in range(len(angle_list)):
                        if angle_list[angle_id] == -1:
                            angle_list[angle_id] = default_value
                    if self.verbose >= 2:
                        print(f'Substituting missing angles with default value ({default_value}), angles: {angle_list}')
                break
        return angle_list

    @staticmethod
    def ikpy_to_degree_conversion(angle_list: list) -> list:
        new_angle_list = []
        for value in angle_list[1:5]:
            new_angle_list.append(np.rad2deg(value) + 90)
        return new_angle_list

    def sub_mode_ik_start_callback(self) -> None:
        # this function is called when the arm is switched to inverse kinematics sub mode

        # self.set_desired_angles(self.get_safe_arm_angle_list(clamped=True, default_value=90))
        # convenient starting position for the gripper
        self.set_desired_angles([90, 45, 35, 35, 90, 90])
        print(f'desired_angle_list: {self.desired_angle_list}')

        self.gripper_speed = [0, 0, 0]
        # intermediate value to calculate initial gripper coordinates
        ikpy_angle_list = np.deg2rad(self.desired_angle_list) - np.pi / 2
        self.gripper_pos = self.servo_chain.forward_kinematics(joints=ikpy_angle_list)[:3, 3]
        print(f'gripper_pos: {self.gripper_pos}')

    def sub_mode_fk_start_callback(self) -> None:
        # this function is called when the arm is switched to forward kinematics sub mode
        self.servo_speed_list = [0, 0, 0, 0, 0, 0]
        self.set_desired_angles(self.get_safe_arm_angle_list(clamped=True, default_value=90))
