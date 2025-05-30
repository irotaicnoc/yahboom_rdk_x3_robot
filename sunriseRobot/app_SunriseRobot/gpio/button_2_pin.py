import Hobot.GPIO as GPIO

import global_constants as gc


class Button2Pin:
    def __init__(self, control_cable: int, callback: callable, mode: str = GPIO.BOARD):
        # Set the pin numbering mode to BOARD (1-40)
        GPIO.setmode(mode)
        self.control_cable = control_cable
        self.pressed = False
        GPIO.setup(self.control_cable, GPIO.IN)

    # def pressed_callback(self, callback: callable):
        print('Button2Pin: waiting for button press...')
        GPIO.add_event_detect(self.control_cable, GPIO.RAISING, callback=callback, bouncetime=200)
        # print()

    def __del__(self):
        GPIO.cleanup(self.control_cable)
        