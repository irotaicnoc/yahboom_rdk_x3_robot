import Hobot.GPIO as GPIO

import global_constants as gc


class Button2Pin:
    def __init__(self, control_cable: int, callback: callable, mode: str = GPIO.BOARD):
        # Set the pin numbering mode to BOARD (1-40)
        GPIO.setmode(mode)
        self.control_cable = control_cable
        GPIO.setup(self.control_cable, GPIO.IN)

        GPIO.add_event_detect(self.control_cable, GPIO.RISING, callback=callback, bouncetime=200)
        GPIO.add_event_callback(self.control_cable, self.rising_detected)

        GPIO.add_event_detect(self.control_cable, GPIO.FALLING, callback=callback, bouncetime=200)
        GPIO.add_event_callback(self.control_cable, self.falling_detected)
        # print()

    @staticmethod
    def rising_detected():
        print('rising detected')

    @staticmethod
    def falling_detected():
        print('falling detected')

    def __del__(self):
        GPIO.cleanup(self.control_cable)
        