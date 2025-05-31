import Hobot.GPIO as GPIO

import global_constants as gc


class Button2Pin:
    def __init__(self, control_cable: int, mode: str = GPIO.BOARD):
        # Set the pin numbering mode to BOARD (1-40)
        # GPIO.setmode(mode)
        self.control_cable = control_cable
        try:
            GPIO.cleanup(self.control_cable)
        except Exception:
            pass
        GPIO.setup(self.control_cable, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # GPIO.add_event_detect(self.control_cable, GPIO.RISING, callback=rising_detected)
        # # GPIO.add_event_callback(self.control_cable, self.rising_detected)
        #
        # GPIO.add_event_detect(self.control_cable, GPIO.FALLING, callback=falling_detected)
        # # GPIO.add_event_callback(self.control_cable, self.falling_detected)
        # # print()

    # def press_listener(self, callback: callable):
    def press_listener(self):
        print(f'listening for button presses on pin {self.control_cable}')
        print(f'GPIO function: {GPIO.gpio_function(self.control_cable)}')
        print('started listening for button presses')
        GPIO.wait_for_edge(self.control_cable, GPIO.FALLING)
        print('rising detected')
        # callback()
        # print('callback executed after falling edge detected')

    def __del__(self):
        GPIO.cleanup(self.control_cable)


def rising_detected():
    print('rising detected')


def falling_detected():
    print('falling detected')