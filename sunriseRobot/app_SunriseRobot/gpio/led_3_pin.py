import Hobot.GPIO as GPIO

import global_constants as gc


class Led3Pin:
    def __init__(self, red_power_cable: int, green_power_cable: int, mode: str = GPIO.BOARD):
        """
        Initialize the Led3Pin with specified GPIO pins for red and green lights.
        """
        # Set the pin numbering mode to BOARD (1-40)
        GPIO.setmode(mode)
        self.COLOR_LIST = [gc.POWER_OFF, gc.RED, gc.GREEN, gc.ORANGE]
        self.red_power_cable = red_power_cable
        self.green_power_cable = green_power_cable
        self.channels = [self.red_power_cable, self.green_power_cable]
        GPIO.cleanup(self.channels)

        # start turned-off
        GPIO.setup(self.channels, GPIO.OUT, initial=GPIO.LOW)
        self.current_color_index = 0

    def set_color(self, color: str):
        assert color in self.COLOR_LIST, (f'color {color} not among valid values.'
                                          f' Possible colors are {self.COLOR_LIST}')

        self.current_color_index = self.COLOR_LIST.index(color)

        # turn off led
        if color == self.COLOR_LIST[0]:
            GPIO.output(self.red_power_cable, GPIO.LOW)
            GPIO.output(self.green_power_cable, GPIO.LOW)

        # red light
        elif color == self.COLOR_LIST[1]:
            GPIO.output(self.red_power_cable, GPIO.HIGH)
            GPIO.output(self.green_power_cable, GPIO.LOW)

        # green light
        elif color == self.COLOR_LIST[2]:
            GPIO.output(self.red_power_cable, GPIO.LOW)
            GPIO.output(self.green_power_cable, GPIO.HIGH)

        # red and green light
        elif color == self.COLOR_LIST[3]:
            GPIO.output(self.red_power_cable, GPIO.HIGH)
            GPIO.output(self.green_power_cable, GPIO.HIGH)

    def next_color(self):
        self.set_color(self.COLOR_LIST[(self.current_color_index + 1) % len(self.COLOR_LIST)])

    def __del__(self):
        self.set_color(gc.POWER_OFF)
        GPIO.cleanup(self.channels)
        