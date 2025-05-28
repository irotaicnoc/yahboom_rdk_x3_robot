import Hobot.GPIO as GPIO

import global_constants as gc


class TriCableLed:
    def __init__(self, red_light: int, green_light: int, shared_cable: int, mode: str = GPIO.BOARD):
        """
        Initialize the TriCableLed with specified GPIO pins for red and green lights.
        """
        # Set the pin numbering mode to BOARD (1-40)
        GPIO.setmode(mode)

        self.COLOR_LIST = [gc.POWER_OFF, 'red', 'green', 'orange']

        # define more useful alias
        self.red_light = red_light
        self.green_light = green_light
        self.shared_cable = shared_cable

        # start turned-off
        GPIO.setup(gc.BLUE_CABLE, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.red_light, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.green_light, GPIO.OUT, initial=GPIO.LOW)
        self.current_color_index = 0

    def set_color(self, color: str):
        assert color in self.COLOR_LIST, (f'color {color} not among valid values.'
                                                 f' Possible colors are {self.COLOR_LIST}')

        self.current_color_index = self.COLOR_LIST.index(color)

        # turn off led
        if color == self.COLOR_LIST[0]:
            GPIO.output(self.red_light, GPIO.LOW)
            GPIO.output(self.green_light, GPIO.LOW)

        # red light
        elif color == self.COLOR_LIST[1]:
            GPIO.output(self.red_light, GPIO.HIGH)
            GPIO.output(self.green_light, GPIO.LOW)

        # green light
        elif color == self.COLOR_LIST[2]:
            GPIO.output(self.red_light, GPIO.LOW)
            GPIO.output(self.green_light, GPIO.HIGH)

        # red and green light
        elif color == self.COLOR_LIST[3]:
            GPIO.output(self.red_light, GPIO.HIGH)
            GPIO.output(self.green_light, GPIO.HIGH)

    def next_color(self):
        self.set_color(self.COLOR_LIST[(self.current_color_index + 1) % len(self.COLOR_LIST)])

    def __del__(self):
        self.set_color(gc.POWER_OFF)
        GPIO.cleanup()
        