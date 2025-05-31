import warnings

import Hobot.GPIO as GPIO

import global_constants as gc


class Led2Pin:
    def __init__(self, power_cable: int, mode: str = GPIO.BOARD):
        """
        Initialize the Led2Pin with a GPIO pins for activating the light.
        """
        try:
            GPIO.getmode()
            if GPIO.getmode() != mode:
                warnings.warn(f'GPIO was in mode {GPIO.getmode()}, but it should be in mode {mode}.'
                              f' Setting GPIO mode to {mode}.')
                GPIO.setmode(mode)
        except Exception:
            warnings.warn(f'GPIO mode was not set. Setting GPIO mode to {mode}.')
            GPIO.setmode(mode)
        self.power_cable = power_cable
        try:
            GPIO.cleanup(self.power_cable)
        except Exception:
            pass

        # start with led turned off
        self.turned_on = False
        GPIO.setup(self.power_cable, GPIO.OUT, initial=GPIO.LOW)

    def toggle_state(self):
        """
        Change the state of the LED.
        """
        self.turned_on = not self.turned_on
        GPIO.output(self.power_cable, GPIO.HIGH if self.turned_on else GPIO.LOW)

    def set_state(self, on: bool):
        """
        Set the state of the LED.
        :param on: True to turn on, False to turn off.
        """
        if self.turned_on != on:
            self.toggle_state()

    def __del__(self):
        self.set_state(on=False)
        GPIO.cleanup(self.power_cable)
