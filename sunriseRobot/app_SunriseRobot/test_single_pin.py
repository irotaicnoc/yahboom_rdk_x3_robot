import time

import Hobot.GPIO as GPIO

import global_constants as gc


def main():
    # Set the pin numbering mode to BOARD
    GPIO.setmode(GPIO.BOARD)

    # Set the pin as output and initialize it to high level
    GPIO.setup(gc.VIOLET_CABLE, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(gc.BLUE_CABLE, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(gc.GREEN_CABLE, GPIO.OUT, initial=GPIO.LOW)

    # Record the current pin state
    red_light = GPIO.HIGH
    green_light = GPIO.LOW
    print("Starting demo now! Press CTRL+C to exit")
    # Loop to control the LED light on and off every 1 second
    counter = 0
    while True:
        time.sleep(1)
        GPIO.output(gc.VIOLET_CABLE, red_light)
        GPIO.output(gc.GREEN_CABLE, green_light)
        if counter % 2 == 0:
            red_light ^= GPIO.HIGH
        if counter % 3 == 0:
            green_light ^= GPIO.HIGH

        counter += 1


if __name__ == '__main__':
    main()
