import time
import warnings

import Hobot.GPIO as GPIO

import global_constants as gc


class Button2Pin:
    def __init__(self,
                 control_cable: int,
                 callback_short_click: callable,
                 callback_long_click: callable = None,
                 button_press_required_time: float = 2,
                 mode: str = GPIO.BOARD,
                 ):
        try:
            gpio_mode = GPIO.getmode()
            if gpio_mode is None:
                GPIO.setmode(mode)
            elif gpio_mode != mode:
                warnings.warn(f'GPIO was in mode {gpio_mode}, but it should be in mode {mode}.'
                              f' Setting GPIO mode to {mode}.')
                GPIO.setmode(mode)
        except Exception:
            GPIO.setmode(mode)

        self.callback_short_click = callback_short_click
        self.callback_long_click = callback_long_click
        self.control_cable = control_cable
        self.button_press_required_time = button_press_required_time
        self.is_pressed = False
        self.pressed_time = 0
        try:
            GPIO.cleanup(self.control_cable)
        except Exception:
            pass
        GPIO.setup(self.control_cable, GPIO.IN, pull_up_down='Pull-up')

    def press_listener(self):
        GPIO.wait_for_edge(self.control_cable, GPIO.FALLING)
        if not self.is_pressed:
            self.pressed_time = time.time()
        else:
            executed_long_callback = False
            if self.callback_long_click is not None:
                if time.time() - self.pressed_time > self.button_press_required_time:
                    self.callback_long_click()
                    executed_long_callback = True
            if not executed_long_callback:
                self.callback_short_click()

        self.is_pressed = not self.is_pressed

    def __del__(self):
        GPIO.cleanup(self.control_cable)
