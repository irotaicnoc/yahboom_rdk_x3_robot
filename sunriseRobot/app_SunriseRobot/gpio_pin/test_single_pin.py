import time

import Hobot.GPIO as GPIO

# import global_constants as gc
from .. import global_constants as gc


def main():
    # Set the pin numbering mode to BOARD
    GPIO.setmode(GPIO.BOARD)
    # Set the pin as output and initialize it to high level
    GPIO.setup(gc.VIOLET_CABLE, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(gc.BLUE_CABLE, GPIO.OUT, initial=GPIO.LOW)

    # Record the current pin state
    curr_value = GPIO.HIGH
    print("Starting demo now! Press CTRL+C to exit")
    try:
        # Loop to control the LED light on and off every 1 second
        while True:
            time.sleep(1)
            GPIO.output(gc.VIOLET_CABLE, curr_value)
            curr_value ^= GPIO.HIGH
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    main()
