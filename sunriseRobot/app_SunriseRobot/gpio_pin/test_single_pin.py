import time

import Hobot.GPIO as GPIO

# Define the GPIO channel used as 38
output_pin = 38  # BOARD code 38


def main():
    # Set the pin numbering mode to BOARD
    GPIO.setmode(GPIO.BOARD)
    # Set the pin as output and initialize it to high level
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    # Record the current pin state
    curr_value = GPIO.HIGH
    print("Starting demo now! Press CTRL+C to exit")
    try:
        # Loop to control the LED light on and off every 1 second
        while True:
            time.sleep(1)
            GPIO.output(output_pin, curr_value)
            curr_value ^= GPIO.HIGH
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    main()
