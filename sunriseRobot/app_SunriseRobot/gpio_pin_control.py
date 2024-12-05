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
        self.turn_off()

    def red(self):
        GPIO.output(self.RED_LIGHT, GPIO.HIGH)
        GPIO.output(self.GREEN_LIGHT, GPIO.LOW)

    def green(self):
        GPIO.output(self.RED_LIGHT, GPIO.LOW)
        GPIO.output(self.GREEN_LIGHT, GPIO.HIGH)

    def red_and_green(self):
        GPIO.output(self.RED_LIGHT, GPIO.HIGH)
        GPIO.output(self.GREEN_LIGHT, GPIO.HIGH)

    def turn_off(self):
        GPIO.output(self.RED_LIGHT, GPIO.LOW)
        GPIO.output(self.GREEN_LIGHT, GPIO.LOW)

    def __del__(self):
        GPIO.cleanup()
        