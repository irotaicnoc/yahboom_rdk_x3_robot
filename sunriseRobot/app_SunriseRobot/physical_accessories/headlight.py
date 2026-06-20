import warnings

import Hobot.GPIO as GPIO

import args
import global_constants as gc


class Headlight:
    """
    Controls an external COB LED strip used to illuminate the scene for the camera in low light.

    The strip is a passive 5V load (two wires: +5V and GND) with no signal input, so it cannot be driven
    by an RC servo pulse. It is powered from the expansion board's 5V output and switched by a MOSFET
    module whose trigger is driven by an RDK X3 GPIO pin (BOARD numbering, set in global_constants.py as
    HEADLIGHT_PIN). The pin is driven with hardware PWM, so the MOSFET passes a variable duty cycle to the
    strip and the brightness is adjustable.

    Wiring: board 5V output -> MOSFET load input, strip -> MOSFET load output, HEADLIGHT_PIN -> MOSFET
    trigger, and the MOSFET trigger ground tied to the RDK X3 ground (a common ground is required). The
    RDK GPIO swings 3.3V, so the MOSFET module must fully switch at a 3.3V trigger.

    Brightness levels are configurable in configs/headlight.yaml because the exact duty -> perceived
    brightness mapping depends on the strip and is best tuned on the robot. `brightness_levels[0]` is
    always the OFF state (duty 0); the remaining entries are the "on" states the light cycles through.
    """

    def __init__(self, pin: int, mode: str = GPIO.BOARD, **kwargs):
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'headlight.yaml', **kwargs)
        self.verbose = parameters['verbose']
        self.pin = pin
        self.brightness_levels = parameters['brightness_levels']
        self.pwm_frequency = parameters['pwm_frequency']
        assert len(self.brightness_levels) >= 1, 'brightness_levels must contain at least the OFF level.'

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

        # start turned off (duty 0). The pin must be set up as an output before creating the PWM.
        self.current_level = 0
        self.is_on = False
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        self.pwm = GPIO.PWM(self.pin, self.pwm_frequency)
        self.pwm.start(self.brightness_levels[0])
        if self.verbose >= 2:
            print(f'Headlight ready on GPIO pin {self.pin} '
                  f'(PWM {self.pwm_frequency} Hz, brightness levels {self.brightness_levels}).')

    def _apply_level(self, level: int) -> None:
        level = level % len(self.brightness_levels)
        self.current_level = level
        self.is_on = level != 0
        duty = self.brightness_levels[level]
        self.pwm.ChangeDutyCycle(duty)
        if self.verbose >= 3:
            print(f'Headlight level {level} -> duty {duty}%.')

    def turn_on(self) -> None:
        # turn on at the first non-off level if there is one, otherwise stay off
        self._apply_level(1 if len(self.brightness_levels) > 1 else 0)

    def turn_off(self) -> None:
        self._apply_level(0)

    def toggle(self) -> None:
        if self.is_on:
            self.turn_off()
        else:
            self.turn_on()

    def set_state(self, on: bool) -> None:
        if on:
            self.turn_on()
        else:
            self.turn_off()

    def next_level(self) -> None:
        """Cycle to the next configured light state (off -> on states -> off)."""
        self._apply_level(self.current_level + 1)

    def __del__(self):
        try:
            self.turn_off()
            self.pwm.stop()
            GPIO.cleanup(self.pin)
        except Exception:
            pass
