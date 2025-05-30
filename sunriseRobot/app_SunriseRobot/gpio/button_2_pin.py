import Hobot.GPIO as GPIO

import global_constants as gc


class Button2Pin:
    def __init__(self, control_cable: int, mode: str = GPIO.BOARD):
        # Set the pin numbering mode to BOARD (1-40)
        GPIO.setmode(mode)
        self.control_cable = control_cable
        self.pressed = False
        GPIO.setup(self.control_cable, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def is_pressed(self):
        """
        Check if the button is pressed.
        :return: True if pressed, False otherwise.
        """
        self.pressed = GPIO.input(self.control_cable) == GPIO.LOW
        return self.pressed

    def __del__(self):
        GPIO.cleanup(self.control_cable)
        