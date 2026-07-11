import time
import warnings
import threading

import Hobot.GPIO as GPIO

import args
import global_constants as gc


class Headlight:
    """
    Controls an external COB LED strip used to illuminate the scene for the camera in low light.

    The strip is a passive 5V load (two wires: +5V and GND) with no signal input, so it cannot be driven
    by an RC servo pulse. It is powered from the expansion board's 5V output and switched by a MOSFET
    module whose trigger is driven by an RDK X3 GPIO pin (BOARD numbering, set in global_constants.py as
    HEADLIGHT_PIN).

    Wiring: board 5V output -> MOSFET load input, strip -> MOSFET load output, HEADLIGHT_PIN -> MOSFET
    trigger, and the MOSFET trigger ground tied to the RDK X3 ground (a common ground is required). The
    RDK GPIO swings 3.3V, so the MOSFET module must fully switch at a 3.3V trigger.

    How the pin drives the MOSFET gate is set by `pwm_mode` in configs/headlight.yaml:
      'onoff'    -> plain digital output, no brightness (any non-zero level is full on). This is the
                    default. Cheap MOSFET trigger modules switch a steady 3.3V reliably; the RDK X3's
                    hardware PWM has a 48 kHz minimum, which is often too fast for such a module's gate to
                    fully charge, leaving the strip dark even though the average duty looks correct.
                    On/off reproduces a steady-DC gate signal, which is known to work with this hardware.
                    In this mode any non-zero brightness level is simply "full on".
      'software' -> best-effort software PWM: a background thread toggles the pin at software_pwm_frequency
                    (a few hundred Hz, low enough for the MOSFET to switch). Gives brightness control but
                    uses CPU and can flicker. Works on any output pin.
      'hardware' -> RDK X3 hardware PWM via GPIO.PWM (PWM-capable pins 32/33 only, min 48 kHz, no CPU cost).
                    Smooth in principle, but many MOSFET modules cannot switch that fast, so the strip may
                    not light. Kept for completeness / faster gate hardware.

    Brightness levels are configurable in configs/headlight.yaml because the exact duty -> perceived
    brightness mapping depends on the strip and is best tuned on the robot. `brightness_levels[0]` is
    always the OFF state (duty 0); the remaining entries are the "on" states the light cycles through.
    """

    def __init__(self, pin: int, mode: str = GPIO.BOARD, **kwargs):
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'headlight.yaml', **kwargs)
        self.verbose = parameters['verbose']
        self.pin = pin
        self.pwm_mode = parameters['pwm_mode']
        self.pwm_frequency = parameters['pwm_frequency']
        self.software_pwm_frequency = parameters['software_pwm_frequency']
        self.brightness_levels = parameters['brightness_levels']
        assert len(self.brightness_levels) >= 1, 'brightness_levels must contain at least the OFF level.'
        assert self.pwm_mode in ('hardware', 'software', 'onoff'), \
            f'Unknown pwm_mode "{self.pwm_mode}" (expected hardware/software/onoff).'

        self.current_level = 0
        self.is_on = False
        self.pwm = None
        # software-PWM state (only used when pwm_mode == 'software')
        self._sw_duty = 0
        self._sw_running = False
        self._sw_thread = None

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

        # Start turned off: brightness_levels[0] is duty 0, so the strip is dark until a level is applied.
        if self.pwm_mode == 'hardware':
            # Hobot.GPIO has no software PWM: a hardware-PWM pin is driven by GPIO.PWM() directly, and the
            # PWM object owns the pin. We must NOT call GPIO.setup() on it first; doing so claims the channel
            # as a plain GPIO output and GPIO.PWM() then raises "This channel is in use".
            self.pwm = GPIO.PWM(self.pin, self.pwm_frequency)
            self.pwm.start(self.brightness_levels[0])
        elif self.pwm_mode == 'software':
            GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
            self._sw_running = True
            self._sw_thread = threading.Thread(
                target=self._software_pwm_loop, name='headlight_software_pwm', daemon=True)
            self._sw_thread.start()
        else:  # onoff
            GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)

        if self.verbose >= 2:
            print(f'Headlight ready on GPIO pin {self.pin} (mode {self.pwm_mode}, brightness levels '
                  f'{self.brightness_levels}).')

    def _software_pwm_loop(self) -> None:
        """Best-effort software PWM: toggle the pin at software_pwm_frequency with the current duty."""
        period = 1.0 / self.software_pwm_frequency
        while self._sw_running:
            duty = self._sw_duty / 100.0
            if duty <= 0:
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(period)
            elif duty >= 1:
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(period)
            else:
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(period * duty)
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(period * (1.0 - duty))

    def _apply_level(self, level: int) -> None:
        level = level % len(self.brightness_levels)
        self.current_level = level
        self.is_on = level != 0
        duty = self.brightness_levels[level]
        if self.pwm_mode == 'hardware':
            self.pwm.ChangeDutyCycle(duty)
        elif self.pwm_mode == 'software':
            self._sw_duty = duty
        else:  # onoff
            GPIO.output(self.pin, GPIO.HIGH if duty > 0 else GPIO.LOW)
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
            self._sw_running = False
            self.turn_off()
            if self.pwm is not None:
                self.pwm.stop()
            GPIO.cleanup(self.pin)
        except Exception:
            pass
