import warnings

import Hobot.GPIO as GPIO

import args
import global_constants as gc


class Headlight:
    """
    Controls an external RC LED light bar used to illuminate the scene for the camera in low light.

    The light bar has three wires: power and ground go directly to the battery, while the control
    cable is driven by the robot with a standard RC servo PWM signal (~50 Hz). The light bar's
    onboard controller reads the pulse width to switch on/off (and, depending on the unit, set
    brightness or cycle modes).

    On the RDK X3, Hobot.GPIO supports hardware PWM ONLY on BOARD pins 32 (PWM1) and 33 (PWM0), so
    the control cable must be wired to one of those (see gc.HEADLIGHT_CONTROL_CABLE).

    Pulse widths are configurable in configs/headlight.yaml because the exact pulse-width -> behavior
    mapping depends on the specific light bar and must be tuned on the robot. `pulse_us_levels[0]` is
    always the OFF state; the remaining entries are the "on" states the light can cycle through.
    """

    def __init__(self, control_cable: int, mode: str = GPIO.BOARD, **kwargs):
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'headlight.yaml', **kwargs)
        self.verbose = parameters['verbose']
        self.control_cable = control_cable
        self.pwm_frequency_hz = parameters['pwm_frequency_hz']
        self.period_us = 1_000_000 / self.pwm_frequency_hz
        self.pulse_us_levels = parameters['pulse_us_levels']
        assert len(self.pulse_us_levels) >= 1, 'pulse_us_levels must contain at least the OFF pulse.'

        # make sure the GPIO library is in the expected pin-numbering mode (mirrors the gpio/ classes)
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

        GPIO.setup(self.control_cable, GPIO.OUT, initial=GPIO.LOW)
        self.pwm = GPIO.PWM(self.control_cable, self.pwm_frequency_hz)

        # start turned off, but keep emitting the OFF pulse so the light bar always sees a valid signal
        self.current_level = 0
        self.is_on = False
        self.pwm.start(self._pulse_us_to_duty(self.pulse_us_levels[0]))
        if self.verbose >= 2:
            print(f'Headlight ready on BOARD pin {self.control_cable} '
                  f'({self.pwm_frequency_hz} Hz, levels {self.pulse_us_levels} us).')

    def _pulse_us_to_duty(self, pulse_us: float) -> float:
        """Convert an RC pulse width (microseconds) to a PWM duty cycle (0-100 %)."""
        return max(0.0, min(100.0, pulse_us / self.period_us * 100.0))

    def _apply_level(self, level: int) -> None:
        level = level % len(self.pulse_us_levels)
        self.current_level = level
        self.is_on = level != 0
        duty = self._pulse_us_to_duty(self.pulse_us_levels[level])
        self.pwm.ChangeDutyCycle(duty)
        if self.verbose >= 3:
            print(f'Headlight level {level} -> {self.pulse_us_levels[level]} us ({duty:.1f} % duty).')

    def turn_on(self) -> None:
        # turn on at the first non-off level if there is one, otherwise stay off
        self._apply_level(1 if len(self.pulse_us_levels) > 1 else 0)

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
            GPIO.cleanup(self.control_cable)
        except Exception:
            pass
