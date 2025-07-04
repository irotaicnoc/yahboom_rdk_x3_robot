import warnings


class AvailableFunctions:
    """
    This class contains the robot functions available for function calls with voice interaction.
    """

    def __init__(self, robot_body, robot_head, arm=None, light=None, verbose: int = 0):
        """
        Initializes the AvailableFunctions class with the modules from which the functions come.
        """
        self.robot_body = robot_body
        self.robot_head = robot_head
        if arm is not None:
            self.arm = arm
        if light is not None:
            self.light = light
        self.verbose = verbose
        self.EXCLUDED_METHODS = [
            'add_mode_callback',
            'add_sub_mode_callback',
            'all_sub_modes',
            'toggle_gui_mode',
            'toggle_ros2_vr_connection',
            'deactivate_ros2',
            'activate_ros2',
            'toggle_hotspot',
            'deactivate_hotspot',
            'activate_hotspot',
            'next_vision_model',
            'previous_vision_model',
            'next_target',
            'previous_target',
            # below are methods that are not callable for now, but could be added in the future
            'next_sub_mode',
            'toggle_lidar_listener',
            'set_movement',
        ]

    def __getattr__(self, item):
        """
        This method is called when an attribute is not found in the instance.
        It allows access to methods of the robot_head that are not explicitly defined in this class.
        """
        if hasattr(self.robot_head, item):
            if not item.startswith('_') and callable(getattr(self.robot_head, item)):
                if item not in self.EXCLUDED_METHODS:
                    return getattr(self.robot_head, item)
                else:
                    if self.verbose >= 1:
                        warnings.warn(f'Trying to access excluded method "{item}".')
        raise AttributeError(f'"{self.__class__.__name__}" object has no attribute "{item}"')

    def beep(self, seconds: float) -> None:
        seconds = round(seconds, 2)
        seconds = min(max(seconds, 0), 5)
        self.robot_body.set_beep(on_time=seconds * 1000)

    # def set_arm_motors_state(self, rigid: bool) -> None:
    #     if self.arm is None:
    #         raise ValueError('Arm module is not initialized.')
    #     self.arm.toggle_rigid(rigid=rigid)

    def set_arm_joint_angles(self,
                             base_rotation: int = None,
                             base_inclination: int = None,
                             elbow_1_inclination: int = None,
                             elbow_2_inclination: int = None,
                             gripper_rotation: int = None,
                             gripper_opening: int = None,
                             ) -> None:
        if self.arm is None:
            raise ValueError('Arm module is not initialized.')
        # for the robot, gripper_opening=0 means fully open, gripper_opening=180 means fully closed.
        # however, gemini expects the opposite, so we need to invert the value.
        if gripper_opening is not None:
            gripper_opening = 180 - gripper_opening
        self.arm.set_desired_angles(angle_list=[base_rotation,
                                                base_inclination,
                                                elbow_1_inclination,
                                                elbow_2_inclination,
                                                gripper_rotation,
                                                gripper_opening])

    # def change_light_effect(self) -> None:
    #     if self.light is None:
    #         raise ValueError('Internal light module is not initialized.')
    #     self.light.next_light_effect()
    #
    # def turn_off_lights(self) -> None:
    #     if self.light is None:
    #         raise ValueError('Internal light module is not initialized.')
    #     self.light.stop()

    def move_arm(self, x_axis: float = None, y_axis: float = None, z_axis: float = None) -> None:
        """
        Moves the point (gripper) of the arm along the specified axes.
        :param x_axis: Movement along the X-axis (left/right).
        :param y_axis: Movement along the Y-axis (forward/backward).
        :param z_axis: Movement along the Z-axis (up/down).
        """
        if self.arm is None:
            raise ValueError('Arm module is not initialized.')
        # the number should be smaller, hence the "/ 5"
        current_gripper_pos = self.arm.get_gripper_position()
        if x_axis is not None:
            current_gripper_pos[0] += x_axis / 5
        # left and right are inverted
        if y_axis is not None:
            current_gripper_pos[1] -= y_axis / 5
        if z_axis is not None:
            current_gripper_pos[2] += z_axis / 5
        self.arm.set_gripper_position(current_gripper_pos)

    def control_gripper(self, rotation: float = None, opening: bool = None) -> None:
        """
        Rotates and opens/closes the gripper.
        :param rotation: The rotation of the gripper, between 0 and 180 degrees.
        :param opening: The opening of the gripper. If True, the gripper is fully open, if False, it is fully closed.
        """
        if self.arm is None:
            raise ValueError('Arm module is not initialized.')
        # Convert opening to degrees (0-180)
        opening_degrees = None
        if opening is not None:
            if opening:
                opening_degrees = 180
            else:
                opening_degrees = 0
        self.arm.set_gripper_state(gripper_rotation=rotation, gripper_opening=opening_degrees)
