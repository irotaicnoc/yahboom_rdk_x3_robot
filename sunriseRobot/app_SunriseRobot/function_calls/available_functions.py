

class AvailableFunctions:
    """
    This class contains the robot functions available for function calls with voice interaction.
    """

    def __init__(self, robot_body, robot_head, arm=None, light=None):
        """
        Initializes the AvailableFunctions class with the modules from which the functions come.
        """
        self.robot_body = robot_body
        if arm is not None:
            self.arm = arm
        if light is not None:
            self.light = light

        for attr in dir(robot_head):
            if not attr.startswith('_') and callable(getattr(robot_head, attr)):
                if attr not in ['add_mode_callback', 'add_sub_mode_callback', 'all_sub_modes']:
                    locals()[attr] = getattr(robot_head, attr)

    def beep(self, seconds: float):
        seconds = round(seconds, 2)
        self.robot_body.set_beep(on_time=seconds * 1000)

    def set_arm_state(self, rigid: bool):
        if self.arm is None:
            raise ValueError("Arm module is not initialized.")
        self.arm.toggle_rigid(rigid=rigid)

    def set_arm_angles(self, angles: list):
        if self.arm is None:
            raise ValueError("Arm module is not initialized.")
        self.arm.set_desired_angles(angles=angles)

    def change_light_effect(self):
        if self.light is None:
            raise ValueError("Internal light module is not initialized.")
        self.light.next_light_effect()

    def turn_off_lights(self):
        if self.light is None:
            raise ValueError("Internal light module is not initialized.")
        self.light.stop()
