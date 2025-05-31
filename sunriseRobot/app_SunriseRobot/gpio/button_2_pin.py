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
            GPIO.getmode()
            if GPIO.getmode() != mode:
                warnings.warn(f'GPIO was in mode {GPIO.getmode()}, but it should be in mode {mode}.'
                              f' Setting GPIO mode to {mode}.')
                GPIO.setmode(mode)
        except Exception:
            warnings.warn(f'GPIO mode was not set. Setting GPIO mode to {mode}.')
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

        # GPIO.add_event_detect(self.control_cable, GPIO.RISING, callback=rising_detected)
        # # GPIO.add_event_callback(self.control_cable, self.rising_detected)
        #
        # GPIO.add_event_detect(self.control_cable, GPIO.FALLING, callback=falling_detected)
        # # GPIO.add_event_callback(self.control_cable, self.falling_detected)
        # # print()

    def press_listener(self):
        GPIO.wait_for_edge(self.control_cable, GPIO.FALLING)
        # GPIO.wait_for_edge(self.control_cable, GPIO.BOTH)
        if not self.is_pressed:
            self.pressed_time = time.time()
            print('start counting')
        else:
            executed_long_callback = False
            if self.callback_long_click is not None:
                if time.time() - self.pressed_time > self.button_press_required_time:
                    print(f'executing long click callback (time: {time.time() - self.pressed_time:.2f}s)')
                    self.callback_long_click()
                    executed_long_callback = True
            if not executed_long_callback:
                print(f'executing short click callback (time: {time.time() - self.pressed_time:.2f}s)')
                self.callback_short_click()

        self.is_pressed = not self.is_pressed

    def __del__(self):
        GPIO.cleanup(self.control_cable)
