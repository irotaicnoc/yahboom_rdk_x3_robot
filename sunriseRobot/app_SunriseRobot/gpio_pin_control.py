import Hobot.GPIO as GPIO

import global_constants as gc


class GpioLed:
    def __init__(self):
        # Set the pin numbering mode to BOARD (1-40)
        GPIO.setmode(GPIO.BOARD)

        # define more useful alias
        self.RED_LIGHT = gc.VIOLET_CABLE
        self.GREEN_LIGHT = gc.GREEN_CABLE

        # start turned-off
        GPIO.setup(gc.BLUE_CABLE, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.RED_LIGHT, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.GREEN_LIGHT, GPIO.OUT, initial=GPIO.LOW)
        self.current_color_index = 0

    def set_color(self, color: str):
        assert color in gc.GPIO_LED_COLOR_LIST, (f'color {color} not among valid values.'
                                                 f' Possible colors are {gc.GPIO_LED_COLOR_LIST}')

        self.current_color_index = gc.GPIO_LED_COLOR_LIST.index(color)

        # turn off led
        if color == gc.GPIO_LED_COLOR_LIST[0]:
            GPIO.output(self.RED_LIGHT, GPIO.LOW)
            GPIO.output(self.GREEN_LIGHT, GPIO.LOW)

        # red light
        elif color == gc.GPIO_LED_COLOR_LIST[1]:
            GPIO.output(self.RED_LIGHT, GPIO.HIGH)
            GPIO.output(self.GREEN_LIGHT, GPIO.LOW)

        # green light
        elif color == gc.GPIO_LED_COLOR_LIST[2]:
            GPIO.output(self.RED_LIGHT, GPIO.LOW)
            GPIO.output(self.GREEN_LIGHT, GPIO.HIGH)

        # red and green light
        elif color == gc.GPIO_LED_COLOR_LIST[3]:
            GPIO.output(self.RED_LIGHT, GPIO.HIGH)
            GPIO.output(self.GREEN_LIGHT, GPIO.HIGH)

    def next_color(self):
        self.set_color(gc.GPIO_LED_COLOR_LIST[(self.current_color_index + 1) % len(gc.GPIO_LED_COLOR_LIST)])

    def __del__(self):
        self.set_color('off')
        GPIO.cleanup()
        