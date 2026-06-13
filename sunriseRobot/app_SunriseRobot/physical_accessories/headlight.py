import args
import global_constants as gc


class Headlight:
    """
    Controls an external RC LED light bar used to illuminate the scene for the camera in low light.

    The light bar has three wires and a standard 3-pin RC servo connector (Ground / Voltage / Signal).
    It plugs into one of the expansion board's "PWM servo" ports. Those ports are driven by the
    onboard STM32 microcontroller (the robot body), so the light is controlled over the existing
    serial protocol via robot_body.set_pwm_servo(servo_id, angle), NOT via the RDK X3 GPIO. The
    STM32 turns the angle into a real RC servo PWM pulse and keeps emitting it, so we only need to
    send a command when the state changes.

    Power: the light can be powered from the port's V pin (set the board's "6V8 / Choose 5V" jumper to
    match the bar) or directly from the battery (then run only the signal wire and leave the port's V
    pin disconnected). Either way the code is identical.

    Angles are configurable in configs/headlight.yaml because the exact angle -> behavior mapping
    depends on the specific light bar and must be tuned on the robot. `angle_levels[0]` is always the
    OFF state; the remaining entries are the "on" states the light can cycle through.
    """

    def __init__(self, robot_body, servo_id: int, **kwargs):
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'headlight.yaml', **kwargs)
        self.verbose = parameters['verbose']
        self.robot_body = robot_body
        self.servo_id = servo_id
        self.angle_levels = parameters['angle_levels']
        assert len(self.angle_levels) >= 1, 'angle_levels must contain at least the OFF angle.'

        # start turned off (the STM32 holds the last commanded pulse on its own)
        self.current_level = 0
        self.is_on = False
        self.robot_body.set_pwm_servo(self.servo_id, self.angle_levels[0])
        if self.verbose >= 2:
            print(f'Headlight ready on PWM servo port S{self.servo_id} (angle levels {self.angle_levels}).')

    def _apply_level(self, level: int) -> None:
        level = level % len(self.angle_levels)
        self.current_level = level
        self.is_on = level != 0
        angle = self.angle_levels[level]
        self.robot_body.set_pwm_servo(self.servo_id, angle)
        if self.verbose >= 3:
            print(f'Headlight level {level} -> angle {angle}.')

    def turn_on(self) -> None:
        # turn on at the first non-off level if there is one, otherwise stay off
        self._apply_level(1 if len(self.angle_levels) > 1 else 0)

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
        except Exception:
            pass
