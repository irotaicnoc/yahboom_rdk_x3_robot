import time
import warnings
import numpy as np

import args
import utils
import global_constants as gc
from physical_accessories.arm_kinematics import ArmKinematics


class Arm:
    def __init__(self, robot_head, robot_body, **kwargs):
        self.robot_head = robot_head
        self.robot_body = robot_body
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'arm.yaml', **kwargs)
        self.verbose = parameters['verbose']

        # arm useful positions
        # VERTICAL_POSITION does not change the current gripper opening
        self.VERTICAL_POSITION = [90, 90, 90, 90, 90]
        # FOLDED_POSITION does not change the current gripper opening
        self.FOLDED_POSITION = [90, 180, 0, 0, 90]
        # FORWARD_POSITION also opens the gripper
        self.FORWARD_POSITION = [90, 45, 35, 35, 90, 90]

        arm_initial_angles = self.get_safe_arm_angle_list(clamped=False)
        if arm_initial_angles == [-1, -1, -1, -1, -1, -1]:
            raise Exception('The robotic arm is not connected. Mode "user_controlled (arm_fk)" and'
                            ' "user_controlled (arm_ik)" will not be available')

        robot_head.robot_sub_mode_dict[gc.MODE_USER_CONTROLLED].append(gc.SUB_MODE_ARM_FK)
        self.is_rigid = False
        self.arm_speed_proportion_fk = parameters['arm_speed_proportion_fk']
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

        # set the arm to rigid state and perform all necessary operations
        self.toggle_rigid()

        robot_head.add_sub_mode_callback(
            sub_mode=gc.SUB_MODE_ARM_FK,
            callback=self.sub_mode_fk_start_callback,
            start=True,
        )
        robot_head.add_sub_mode_callback(
            sub_mode=gc.SUB_MODE_ARM_FK,
            callback=self.sub_mode_arm_end_callback,
            start=False,
        )
        robot_head.add_sub_mode_callback(
            sub_mode=gc.SUB_MODE_WHEELS,
            callback=self.sub_mode_wheel_start_callback,
            start=True,
        )

        self.arm_speed_proportion_ik = parameters['arm_speed_proportion_ik']
        # Analytical inverse/forward kinematics tailored to this exact arm. Replaces the previous generic
        # ikpy numerical optimiser (which searched iteratively on every call and made IK mode laggy); the
        # geometry is read from the same urdf/arm.urdf ikpy used. See arm_kinematics.py.
        self.kinematics = ArmKinematics(urdf_path=gc.URDF_FOLDER_PATH + 'arm.urdf', verbose=self.verbose)
        robot_head.robot_sub_mode_dict[gc.MODE_USER_CONTROLLED].append(gc.SUB_MODE_ARM_IK)
        robot_head.add_sub_mode_callback(
            sub_mode=gc.SUB_MODE_ARM_IK,
            callback=self.sub_mode_ik_start_callback,
            start=True,
        )
        robot_head.add_sub_mode_callback(
            sub_mode=gc.SUB_MODE_ARM_IK,
            callback=self.sub_mode_arm_end_callback,
            start=False,
        )
        # these are the coordinates of the gripper in the robot's coordinate system. In inverse kinematics mode
        # they are used in place of the desired angles for motors 0, 1, 2, 3. Only motors 4 (gripper rotation)
        # and 5 (gripper opening) are controlled in the same way in both sub modes.
        # the first value (x) controls the left-right position of the gripper,
        # the second value (y) controls the forward-backward position of the gripper,
        # the third value (z) controls the up-down position of the gripper
        self.gripper_pos = [0, 0, 0]
        # # speed of the gripper in the x, y, z directions
        self.gripper_speed = [0, 0, 0]
        # IK motion is integrated against real elapsed time (see update_desired_angles) so the gripper speed
        # does not depend on the control-loop rate. last_ik_update_time is the previous integration instant;
        # max_ik_dt caps a single step so a stall (mode switch / hitch) cannot turn into one big jump.
        self.last_ik_update_time = time.time()
        self.max_ik_dt = 0.1
        # initializes gripper position and speed, and desired angles
        self.sub_mode_ik_start_callback()

    def toggle_rigid(self, rigid: bool = None) -> None:
        if rigid is not None:
            if self.is_rigid == rigid:
                return

        self.is_rigid = not self.is_rigid
        # block arm at its current position
        if self.is_rigid:
            self.set_desired_angles(angle_list=self.get_safe_arm_angle_list(clamped=True, default_value=90))
        self.robot_body.set_arm_torque(enable=self.is_rigid)
        # beep to signal the change in arm state
        self.robot_body.set_beep(gc.SHORT_BEEP)
        if self.verbose >= 2:
            if self.is_rigid:
                print(f'Arm is rigid')
            else:
                print(f'Arm can be moved manually, but cannot be controlled by the controller')

    def get_gripper_position(self) -> list:
        # returns the current position of the gripper in the robot's coordinate system
        # the first value (x) controls the left-right position of the gripper,
        # the second value (y) controls the forward-backward position of the gripper,
        # the third value (z) controls the up-down position of the gripper
        return self.kinematics.forward_kinematics(self.desired_angle_list[:4])

    def set_gripper_position(self, gripper_pos: list) -> None:
        self.run_time = utils.change_range(
            value=self.robot_head.speed_coefficient,
            original_min=0.1,
            original_max=1,
            new_min=self.arm_automated_speed[1],
            new_max=self.arm_automated_speed[0],
        )
        self.gripper_pos = gripper_pos
        self.desired_angle_list[:4] = self.clamp_angle_list(
            angle_list=self.kinematics.inverse_kinematics(
                target=self.gripper_pos,
                current_angles=self.desired_angle_list[:4],
            ),
            default_value=90,
        )

    def set_gripper_state(self, gripper_rotation: int = None, gripper_opening: int = None) -> None:
        self.run_time = utils.change_range(
            value=self.robot_head.speed_coefficient,
            original_min=0.1,
            original_max=1,
            new_min=self.arm_automated_speed[1],
            new_max=self.arm_automated_speed[0],
        )
        # gripper_rotation is the angle of the gripper rotation servo (motor id=4, physical_label=5)
        # gripper_opening is the angle of the gripper opening servo (motor id=5, physical_label=6)
        if gripper_rotation is not None:
            self.desired_angle_list[4] = np.clip(gripper_rotation, a_min=0, a_max=180)
        if gripper_opening is not None:
            # for the robot, gripper_opening=0 means fully open, gripper_opening=180 means fully closed.
            # however, gemini expects the opposite, so we need to invert the value.
            self.desired_angle_list[5] = np.clip(180 - gripper_opening, a_min=0, a_max=180)

    def set_desired_angles(self, angle_list: list) -> None:
        self.run_time = utils.change_range(
            value=self.robot_head.speed_coefficient,
            original_min=0.1,
            original_max=1,
            new_min=self.arm_automated_speed[1],
            new_max=self.arm_automated_speed[0],
        )
        # it can accept angle lists shorter than 6, and only move the first len(angle_list) servos.
        # In particular, in is useful with lists of length 4 and 5, to ignore the gripper rotation and opening, or just
        # the gripper opening
        # if an angle has value None, it is ignored and the current angle is kept
        for angle_id in range(len(angle_list)):
            angle = angle_list[angle_id]
            if angle is not None:
                self.desired_angle_list[angle_id] = angle_list[angle_id]

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

            # advance the target by the commanded velocity scaled by the real elapsed time, so the motion
            # speed is independent of the control-loop rate (gripper_speed is a velocity in m/s). Without
            # the dt term the gripper moved a fixed step per iteration, so it sped up drastically when the
            # slow ikpy solver was replaced by the instant analytical one and the loop rate jumped.
            now = time.time()
            dt = now - self.last_ik_update_time
            self.last_ik_update_time = now
            dt = min(max(dt, 0.0), self.max_ik_dt)   # guard against a stall producing a big jump

            # only recompute when the gripper is actually moving (the analytical IK is cheap, but there is
            # no point solving for an unchanged pose)
            if self.gripper_speed[0] != 0 or self.gripper_speed[1] != 0 or self.gripper_speed[2] != 0:
                self.gripper_pos[0] += self.gripper_speed[0] * dt
                self.gripper_pos[1] += self.gripper_speed[1] * dt
                self.gripper_pos[2] += self.gripper_speed[2] * dt

                # update the desired angles for motors 0-3 from the new gripper point (motors 4 and 5,
                # gripper rotation and opening, are handled separately below). The analytical solver holds
                # the current gripper pitch while it repositions.
                self.desired_angle_list[:4] = self.clamp_angle_list(
                    angle_list=self.kinematics.inverse_kinematics(
                        target=self.gripper_pos,
                        current_angles=self.desired_angle_list[:4],
                    ),
                    default_value=90,
                )

                # Pull the stored target back onto what the arm can actually reach, by reading back the
                # gripper point of the (clamped) angles we just committed. This is the cheap analytical
                # forward kinematics in software, NOT a hardware query, so it is fine every iteration: in
                # the reachable range it is a no-op (FK(IK(p)) == p), and only at the workspace / servo
                # limits does it act, stopping gripper_pos from running away past the arm. That runaway is
                # what made the arm feel stuck: you had to unwind all the phantom travel before it would
                # move back. (If ever too costly, this could be throttled to every Nth iteration, at the
                # price of letting the target drift up to N steps beyond reach.)
                self.gripper_pos = self.kinematics.forward_kinematics(self.desired_angle_list[:4])

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

    def get_safe_arm_angle_list(self,
                                clamped: bool = True,
                                retry_limit: int = 10,
                                default_value: int = -1,
                                exclude_gripper_opening: bool = False,
                                ) -> list:
        # if exclude_gripper_opening is True, the gripper opening angle is not included in the returned list (len = 5
        # instead of 6)
        if exclude_gripper_opening:
            angle_list = [-1, -1, -1, -1, -1]
        else:
            angle_list = [-1, -1, -1, -1, -1, -1]

        counter = 0
        while -1 in angle_list:
            temp_angle_list = self.robot_body.get_arm_angle_list()
            if exclude_gripper_opening:
                temp_angle_list = temp_angle_list[:5]  # exclude gripper opening angle
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

    def sub_mode_ik_start_callback(self) -> None:
        # this function is called when the arm is switched to inverse kinematics sub mode
        self.toggle_rigid(rigid=True)

        # self.set_desired_angles(self.get_safe_arm_angle_list(clamped=True, default_value=90))
        # convenient starting position for the gripper
        self.set_desired_angles(self.FORWARD_POSITION)

        self.servo_speed_list = [0, 0, 0, 0, 0, 0]
        self.gripper_speed = [0, 0, 0]
        self.gripper_pos = self.get_gripper_position()
        # start the dt clock fresh so the first integration step after entering IK mode is not a stale jump
        self.last_ik_update_time = time.time()

    def sub_mode_fk_start_callback(self) -> None:
        self.toggle_rigid(rigid=True)
        # this function is called when the arm is switched to forward kinematics sub mode
        self.servo_speed_list = [0, 0, 0, 0, 0, 0]
        self.gripper_speed = [0, 0, 0]
        self.set_desired_angles(self.FORWARD_POSITION)
        self.robot_body.set_arm_angle_list(angle_s=self.desired_angle_list, run_time=self.run_time)

    def sub_mode_arm_end_callback(self) -> None:
        # this function is called when any arm sub mode is switched to another sub mode
        # for safety, block the arm, so it doesn't move
        self.toggle_rigid(rigid=True)
        # fold the arm to a safe position, so it doesn't hit against anything
        self.set_desired_angles(self.FOLDED_POSITION)
        self.robot_body.set_arm_angle_list(angle_s=self.desired_angle_list, run_time=self.run_time)
        self.servo_speed_list = [0, 0, 0, 0, 0, 0]
        self.gripper_speed = [0, 0, 0]

    def sub_mode_wheel_start_callback(self) -> None:
        # this function is called when wheel sub mode is switched activated, but only if the arm is present
        # for safety, block the arm, so it doesn't move
        self.toggle_rigid(rigid=True)
        # fold the arm to a safe position, so it doesn't hit against anything
        self.set_desired_angles(self.FOLDED_POSITION)
        self.robot_body.set_arm_angle_list(angle_s=self.desired_angle_list, run_time=self.run_time)
        self.servo_speed_list = [0, 0, 0, 0, 0, 0]
        self.gripper_speed = [0, 0, 0]
