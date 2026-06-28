class RemoteHeadlight:
    """
    Stand-in on the RDK X3 for the camera headlight, which now physically lives on the Jetson Nano.

    The COB LED strip was moved to the Jetson for wiring access, but the RDK X3 stays the single control
    surface: all triggers (controller button, vision auto-dark, voice/mobile/VR function calls) still go
    through RobotHead, which calls these methods. Instead of driving a local GPIO pin, this proxy forwards
    the intent as a command over the bidirectional command channel (EthernetServer.send_command), where the
    real Headlight on the Jetson executes it.

    It exposes the same interface as the old local Headlight (turn_on / turn_off / toggle / set_state /
    next_level). State and the duty/brightness mapping live on the Jetson, so this side only emits intent
    (e.g. "toggle") and the Jetson's Headlight flips its own state accordingly.

    The command sender is bound after the EthernetServer is created (see main_thread.py); until then, or if
    the server failed to start, commands are dropped (the same graceful degradation as headlight=None).
    """

    # command names understood by the Jetson's command dispatcher (see voice_robot_interaction main_thread)
    _TURN_ON = 'headlight_turn_on'
    _TURN_OFF = 'headlight_turn_off'
    _TOGGLE = 'headlight_toggle'
    _SET_STATE = 'headlight_set_state'
    _NEXT_LEVEL = 'headlight_next_level'

    def __init__(self, send_command=None, verbose: int = 0):
        """
        :param send_command: callable(name: str, args: dict | None) that sends a command to the Jetson.
            May be None at construction and set later with bind_sender().
        :param verbose: verbosity level.
        """
        self._send_command = send_command
        self.verbose = verbose
        # Optimistic local mirror of the on/off state. The authoritative state lives on the Jetson, but
        # since the RDK X3 is the only controller, mirroring intent here is enough for toggle() and for
        # callers that read is_on (e.g. RobotHead.toggle_headlight's log line).
        self.is_on = False

    def bind_sender(self, send_command) -> None:
        """Wire up the command sender once the ethernet server exists."""
        self._send_command = send_command

    def _send(self, name: str, args=None) -> None:
        if self._send_command is None:
            if self.verbose >= 2:
                print(f'RemoteHeadlight: no command channel yet, dropping "{name}".')
            return
        self._send_command(name, args)

    def turn_on(self) -> None:
        self.is_on = True
        self._send(self._TURN_ON)

    def turn_off(self) -> None:
        self.is_on = False
        self._send(self._TURN_OFF)

    def toggle(self) -> None:
        self.is_on = not self.is_on
        self._send(self._TOGGLE)

    def set_state(self, on: bool) -> None:
        self.is_on = bool(on)
        self._send(self._SET_STATE, {'on': bool(on)})

    def next_level(self) -> None:
        # brightness-level state lives on the Jetson; this side only mirrors on/off optimistically
        self.is_on = True
        self._send(self._NEXT_LEVEL)
