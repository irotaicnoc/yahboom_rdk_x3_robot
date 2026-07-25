import os
import time
import warnings
import threading
from pathlib import Path
from collections import deque

import args
import utils
import global_constants as gc


class RobotHead:
    def __init__(self, **kwargs):
        self.robot_body = kwargs['robot_body']
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'robot_head.yaml', **kwargs)
        self.gui_mode = parameters['gui_mode']
        self.verbose = parameters['verbose']

        # Serializes the *compound* state changes on this object: mode/sub-mode switching, controller
        # registration and voice-session start/stop. Those are read-modify-write or check-then-act sequences
        # (read a value, decide, write it back) reachable from several threads at once: the PS2 controller
        # thread, the VR controller thread, the GPIO button threads and the ethernet server. The GIL makes a
        # single attribute access atomic but not a sequence of them, so without this lock two overlapping
        # calls can interleave (a lost controller count, or start/end callbacks fired for a mode that is no
        # longer current).
        # Plain single-attribute reads (robot_mode, speed_x, ...) are atomic on their own and deliberately
        # stay lock-free, so the hot control loop never blocks on this.
        # Reentrant (RLock) because the mode callbacks are fired while holding it and may call back in here.
        # Lock ordering: this lock is always taken *before* RobotBody._serial_lock (mode callbacks send UART
        # frames), never the other way round, so the two cannot deadlock.
        self._state_lock = threading.RLock()

        # controller parameters
        self.controller_id_list = []
        self.connected_controllers = 0
        self.button_press_required_time = parameters['button_press_required_time']
        self.one_time_check = {}
        self.button_press_timestamp = {}

        # autonomous mode parameters
        self.robot_mode_list = [gc.MODE_USER_CONTROLLED]
        self.robot_sub_mode_dict = {self.robot_mode_list[0]: [gc.SUB_MODE_WHEELS]}
        self.robot_mode = self.robot_mode_list[0]
        self.robot_sub_mode = self.robot_sub_mode_dict[self.robot_mode][0]
        self.mode_start_callbacks = {}
        self.mode_end_callbacks = {}
        self.sub_mode_start_callbacks = {}
        self.sub_mode_end_callbacks = {}
        if self.verbose >= 1:
            print(f'Robot mode: {self.robot_mode} ({self.robot_sub_mode})')
        self.tracking_target_list = parameters['tracking_target_list']
        self.tracking_target_pos = 0
        # search for vision models in the vision model folder
        self.vision_model_list = []
        self.vision_model_pos = 0
        vision_model_folder_path = Path(gc.GENERIC_MODEL_FOLDER_PATH)
        # if it is present, start with the fastest model
        counter = 0
        for vision_model_path in vision_model_folder_path.glob('*.*'):
            if 'yolo11s_640_480_edgetpu' in vision_model_path.name:
                self.vision_model_pos = counter
            self.vision_model_list.append(vision_model_path.name)
            counter += 1
        if self.verbose >= 2:
            print(f'Among vision models: {self.vision_model_list}.')
            print(f'Selecting position: {self.vision_model_pos}.')
            print(f'Starting with vision model: "{self.vision_model_list[self.vision_model_pos]}".')

        # long processes status
        self.ros2_vr_connection_status = 'inactive'
        self.hotspot_status = 'inactive'
        self.hotspot_ip = parameters['hotspot_ip']
        self.lidar_listener_status = 'inactive'

        # motion parameters
        self.steer_speed_proportion = parameters['steer_speed_proportion']
        self.speed_coefficient = parameters['speed_coefficient']
        self.speed_x = 0
        self.speed_y = 0
        self.speed_z = 0
        # used by set_movement_with_duration to stop the robot after a certain time
        self.stop_timestamp = None

        # buzzer, leds, and lights
        self.buzzer_is_active = False
        self.buzzer_state_changed = True

        # voice interaction push-to-talk session (South button). While a session is active the RDK X3
        # forwards mic audio to the Jetson voice interaction: the robot's ReSpeaker for a joystick press
        # ('robot'), or the app phone/headset mic (arriving on /audio_from_vr) for an app press ('app').
        self.voice_session_active = False
        self.voice_session_source = None  # 'robot' | 'app' while a session is active, else None
        # App mic frames handed over by VrAudioSubscriber, waiting to be forwarded to the Jetson during an
        # app session. Bounded so a stalled bridge cannot grow it without limit (oldest frames are dropped).
        self.app_mic_frames = deque(maxlen=64)
        # Latched source of the most recent voice session, kept after the session ends. The TTS response comes
        # back from the Jetson (source-agnostic) seconds later on the speaker bridge, and this is what tells the
        # RDK X3 where to send it: 'robot' -> the robot's own speaker, 'app' -> the connected app only. Unlike
        # voice_session_source it is NOT cleared on stop; it stays until the next session latches a new value.
        # Robust to short taps the Jetson silently drops (they make no TTS to misroute); the only imperfect case
        # is a second, different-source session before the first answer returns, rare for a single user.
        self.last_voice_source = None  # 'robot' | 'app'
        # Whole TTS clips (24 kHz mono int16 PCM) routed to the connected app instead of the robot speaker.
        # Filled by AudioBridgeServer when an app-originated response arrives; drained by TtsToAppPublisher,
        # which chunks and publishes them on /audio_to_app. A response is a single clip, so this only ever holds
        # one (rarely a couple); the cap just guards against growth if the app vanishes mid-playback.
        self.tts_to_app_clips = deque(maxlen=8)

        self.internal_light = parameters['internal_light']
        self.led_3_pin = parameters['led_3_pin']
        # external RC LED light bar (headlight for the camera). May be None if unavailable.
        self.headlight = parameters.get('headlight')

    def start_voice_session(self, source: str) -> None:
        # Push-to-talk pressed (South button / app button A). First-one-wins: if a session from another
        # source is already active, ignore this one so a joystick and an app press cannot fight over the
        # single mic bridge to the Jetson.
        # Locked: the joystick press arrives on the PS2 thread and the app press on the VR controller thread,
        # so without it both could pass the "is a session already active?" check and each set its own source,
        # leaving voice_session_source disagreeing with what the mic bridge is actually streaming.
        with self._state_lock:
            if self.voice_session_active:
                return
            self.voice_session_source = source
            # Latch the source for the whole interaction so the eventual TTS response is routed back to it.
            self.last_voice_source = source
            self.voice_session_active = True
        if self.verbose >= 2:
            print(f'Voice session started (source: {source})')

    def stop_voice_session(self, source: str) -> None:
        # Only the source that owns the active session may end it, so releasing the other button (e.g. an
        # app button-A release while the joystick owns the session) does not cut the session short.
        # Locked for the same reason as start_voice_session: the ownership check and the clearing of the
        # three pieces of session state have to happen as one step.
        with self._state_lock:
            if not self.voice_session_active or self.voice_session_source != source:
                return
            self.voice_session_active = False
            self.voice_session_source = None
            # Drop any app audio that was not forwarded so it cannot leak into the next session.
            self.app_mic_frames.clear()
        if self.verbose >= 2:
            print('Voice session stopped')

    def register_controller(self, controller_id: int) -> bool:
        """
        Record a controller as connected.
        The membership check and the counter increment have to be a single atomic step: both the PS2 thread
        and the VR controller thread call this, and "connected_controllers += 1" is a read-modify-write that
        can lose an update. A lost increment leaves the counter drifting, and once it reaches 0 the control
        loop concludes no controller is attached and stops driving the robot (see ControllerLoop).
        :param controller_id: id of the controller that connected.
        :return: True if the controller was registered, False if that id was already registered.
        """
        with self._state_lock:
            if controller_id in self.controller_id_list:
                return False
            self.controller_id_list.append(controller_id)
            self.connected_controllers += 1
            return True

    def unregister_controller(self, controller_id: int) -> bool:
        """
        Record a controller as disconnected. Counterpart of register_controller, atomic for the same reason.
        :param controller_id: id of the controller that disconnected.
        :return: True if the controller was removed, False if that id was not registered.
        """
        with self._state_lock:
            if controller_id not in self.controller_id_list:
                return False
            self.controller_id_list.remove(controller_id)
            self.connected_controllers -= 1
            return True

    def remove_mode(self, mode: str) -> None:
        """
        Remove a mode from the list of available modes, falling back to the first remaining mode if the robot
        was currently in it. Used when a subsystem fails at runtime and its mode can no longer be entered
        (e.g. the vision agent crashing removes MODE_AUTONOMOUS_VISION).
        Takes the same lock as next_mode: that method reads an index into robot_mode_list and then indexes
        back into it, so a removal landing between the two would raise IndexError.
        :param mode: the mode to remove. Does nothing if it is not in the list.
        """
        with self._state_lock:
            if mode not in self.robot_mode_list:
                return
            self.robot_mode_list.remove(mode)
            if self.robot_mode == mode:
                self.robot_mode = self.robot_mode_list[0]
                if self.robot_sub_mode_dict[self.robot_mode] is not None:
                    self.robot_sub_mode = self.robot_sub_mode_dict[self.robot_mode][0]

    def next_mode(self) -> None:
        if self.verbose >= 3:
            print(f'Switching from {self.robot_mode} ({self.robot_sub_mode}) mode')
        # The whole transition is held under the lock, callbacks included, not just the assignment of
        # robot_mode. Mode switching is reachable from the PS2 thread, the VR controller thread and the GPIO
        # button thread; if two calls interleaved, one could fire the start callbacks of a mode the other has
        # already left, leaving the hardware (arm rigid state, lights) configured for the wrong mode. The
        # callbacks are short (they set fields and send a few UART frames), so the lock is never held long.
        with self._state_lock:
            self.led_3_pin.set_color(gc.POWER_OFF)
            self.internal_light.stop()
            previous_mode = self.robot_mode
            previous_sub_mode = self.robot_sub_mode
            self.robot_mode = self.robot_mode_list[
                (self.robot_mode_list.index(self.robot_mode) + 1) % len(self.robot_mode_list)
            ]
            if self.robot_mode in self.robot_sub_mode_dict:
                self.robot_sub_mode = self.robot_sub_mode_dict[self.robot_mode][0]
            else:
                self.robot_sub_mode = None

            # mode callbacks
            # if the previous mode has callbacks to call at the end, call them. But only if the mode has actually
            # changed. For example if the list has only 1 element, the callback should not be called, because the
            # robot was already in the same mode. Or if the new mode fails to be set and the previous mode is set
            # again
            if previous_mode != self.robot_mode:
                if previous_mode in self.mode_end_callbacks:
                    for callback in self.mode_end_callbacks[previous_mode]:
                        callback()
                # if the new mode has callbacks to call at the start, call them. But only if the mode has actually
                # changed
                if self.robot_mode in self.mode_start_callbacks:
                    for callback in self.mode_start_callbacks[self.robot_mode]:
                        callback()

            # sub mode callbacks
            # if the sub mode also have callbacks to call at the start and end, call them. But only if the mode
            # current mode has a sub mode, and the sub mode actually changed
            # end callbacks
            if previous_sub_mode is not None and self.robot_sub_mode != previous_sub_mode:
                if previous_sub_mode in self.sub_mode_end_callbacks:
                    for callback in self.sub_mode_end_callbacks[previous_sub_mode]:
                        callback()
            # start callbacks
            if self.robot_sub_mode is not None and self.robot_sub_mode != previous_sub_mode:
                if self.robot_sub_mode in self.sub_mode_start_callbacks:
                    for callback in self.sub_mode_start_callbacks[self.robot_sub_mode]:
                        callback()

            # notify the user about the mode change
            self.robot_body.set_beep(gc.MEDIUM_BEEP)
        if self.verbose >= 1:
            print(f'Switching to {self.robot_mode} ({self.robot_sub_mode}) mode')

    def next_sub_mode(self) -> None:
        if self.verbose >= 3:
            print(f'Switching from {self.robot_sub_mode} sub mode')
        # Held under the same lock as next_mode, and for the same reason: this is a read-modify-write on
        # robot_sub_mode followed by callbacks that reconfigure the hardware. It also has to be mutually
        # exclusive with next_mode itself, which writes both robot_mode and robot_sub_mode.
        with self._state_lock:
            if self.robot_mode not in self.robot_sub_mode_dict:
                assert self.robot_sub_mode is None, f'Robot mode {self.robot_mode} does not have sub modes, ' \
                    f'but current sub mode is {self.robot_sub_mode}'
                if self.verbose >= 3:
                    print(f'No sub modes available for {self.robot_mode} mode')
                return

            self.led_3_pin.set_color(gc.POWER_OFF)
            self.internal_light.stop()

            current_sub_mode_list = self.robot_sub_mode_dict[self.robot_mode]
            if current_sub_mode_list is not None and len(current_sub_mode_list) > 0:
                previous_sub_mode = self.robot_sub_mode
                self.robot_sub_mode = current_sub_mode_list[
                    (current_sub_mode_list.index(self.robot_sub_mode) + 1) % len(current_sub_mode_list)
                ]
                # if the sub mode also have callbacks to call at the start and end, call them. But only if the sub
                # mode has actually changed. For example if the list has only 1 element, the callbacks should not be
                # called, because the robot was already in the same sub mode. Or if the new sub mode fails to be set
                # and the previous sub mode is set again.
                if previous_sub_mode != self.robot_sub_mode:
                    # end callbacks
                    if previous_sub_mode in self.sub_mode_end_callbacks:
                        for callback in self.sub_mode_end_callbacks[previous_sub_mode]:
                            callback()
                    if self.robot_sub_mode in self.sub_mode_start_callbacks:
                        for callback in self.sub_mode_start_callbacks[self.robot_sub_mode]:
                            callback()
            else:
                assert self.robot_sub_mode is None, f'Robot mode {self.robot_mode} does not have sub modes, ' \
                                                    f'but current sub mode is {self.robot_sub_mode}'

            # notify the user about the sub mode change
            self.robot_body.set_beep(gc.SHORT_BEEP)
        if self.verbose >= 1:
            print(f'Switching to {self.robot_sub_mode} sub mode')

    def next_target(self) -> None:
        self.tracking_target_pos += 1
        self.tracking_target_pos = self.tracking_target_pos % len(self.tracking_target_list)
        if self.verbose >= 1:
            print(f'Switching to target: {self.tracking_target_list[self.tracking_target_pos]}')

    def previous_target(self) -> None:
        self.tracking_target_pos -= 1
        self.tracking_target_pos = self.tracking_target_pos % len(self.tracking_target_list)
        if self.verbose >= 1:
            print(f'Switching to target: {self.tracking_target_list[self.tracking_target_pos]}')

    def set_target(self, target: str) -> None:
        """
        Set the current target to a specific one from the tracking target list.
        :param target: The target to set, must be in the tracking_target_list.
        """
        if target not in self.tracking_target_list:
            raise ValueError(f'Target "{target}" is not in the tracking target list: {self.tracking_target_list}')
        self.tracking_target_pos = self.tracking_target_list.index(target)
        if self.verbose >= 1:
            print(f'Switching to target: {self.tracking_target_list[self.tracking_target_pos]}')

    def next_vision_model(self) -> None:
        self.vision_model_pos += 1
        self.vision_model_pos = self.vision_model_pos % len(self.vision_model_list)
        if self.verbose >= 1:
            print(f'Switching to vision model: {self.vision_model_list[self.vision_model_pos]}')

    def previous_vision_model(self) -> None:
        self.vision_model_pos -= 1
        self.vision_model_pos = self.vision_model_pos % len(self.vision_model_list)
        if self.verbose >= 1:
            print(f'Switching to vision model: {self.vision_model_list[self.vision_model_pos]}')

    def increase_speed_coefficient(self) -> None:
        self.speed_coefficient = min(1.0, self.speed_coefficient + 0.1)
        if self.verbose >= 2:
            print(f'Speed coefficient: {self.speed_coefficient}')

    def decrease_speed_coefficient(self) -> None:
        self.speed_coefficient = max(0.1, self.speed_coefficient - 0.1)
        if self.verbose >= 2:
            print(f'Speed coefficient: {self.speed_coefficient}')

    def activate_hotspot(self) -> None:
        if self.hotspot_status == 'active':
            if self.verbose >= 2:
                print('Hotspot is already active')
            return
        self.hotspot_status = 'processing'
        utils.start_generic_process(robot_head=self, name='Starting hotspot')
        os.system('sleep 2')
        os.system('systemctl stop wpa_supplicant')
        os.system('ip addr flush dev wlan0')
        os.system('sleep 0.5')
        os.system('ifconfig wlan0 down')
        os.system('sleep 1')
        os.system('ifconfig wlan0 up')
        os.system(f'hostapd -B {gc.MAIN_FOLDER_PATH}hotspot/etc/hostapd.conf')
        os.system(f'ifconfig wlan0 {self.hotspot_ip} netmask 255.255.255.0')
        os.system('systemctl start isc-dhcp-server')
        self.hotspot_status = 'active'
        utils.finish_generic_process(robot_head=self)

    def deactivate_hotspot(self) -> None:
        if self.hotspot_status == 'inactive':
            if self.verbose >= 2:
                print('Hotspot is already inactive')
            return
        self.hotspot_status = 'processing'
        utils.start_generic_process(robot_head=self, name='Stopping hotspot')
        utils.kill_process_(process_name='hostapd', verbose=self.verbose)
        os.system('systemctl stop isc-dhcp-server')
        os.system('ip addr flush dev wlan0')
        os.system('sleep 0.5')
        os.system('ifconfig wlan0 down')
        os.system('sleep 1')
        os.system('ifconfig wlan0 up')
        os.system('systemctl start wpa_supplicant')
        self.hotspot_status = 'inactive'
        utils.finish_generic_process(robot_head=self)

    def toggle_hotspot(self) -> None:
        if self.hotspot_status == 'inactive':
            self.activate_hotspot()
        elif self.hotspot_status == 'active':
            self.deactivate_hotspot()
        else:
            print(f'Hotspot is in "{self.hotspot_status}" state. Cannot be changed now')

    def activate_ros2(self) -> None:
        if self.ros2_vr_connection_status == 'active':
            if self.verbose >= 2:
                print('ROS2 VR connection is already active')
            return
        self.ros2_vr_connection_status = 'processing'
        utils.start_generic_process(robot_head=self, name='Starting ROS2')
        if self.gui_mode:
            # activate ROS2 endpoint
            os.system('gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash;cd /root/marco_ros2_ws/;'
                      'source install/local_setup.bash;ros2 launch ros_tcp_endpoint endpoint_launch.py;exec bash"')
            # activate lidar publisher
            os.system('gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash;'
                      'source /root/marco_ros2_ws/install/setup.bash;'
                      'ros2 launch lidar_pub ms200_scan.launch.py;exec bash"')
        else:
            os.system(f'{gc.SCRIPT_FOLDER_PATH}start_ros2_no_gui.sh')
        self.ros2_vr_connection_status = 'active'
        utils.finish_generic_process(robot_head=self)

    def deactivate_ros2(self) -> None:
        # TODO: it does not really kill the process in the separate console
        if self.ros2_vr_connection_status == 'inactive':
            if self.verbose >= 2:
                print('ROS2 VR connection is already inactive')
            return
        self.ros2_vr_connection_status = 'processing'
        utils.start_generic_process(robot_head=self, name='Stopping ROS2')
        # rclpy.shutdown()
        # deactivate ROS2 endpoint
        utils.kill_process_(process_name='ros2', verbose=self.verbose)
        # deactivate lidar publisher
        utils.kill_process_(process_name='lidar', verbose=self.verbose)
        self.led_3_pin.set_color(gc.GREEN)
        self.ros2_vr_connection_status = 'inactive'
        utils.finish_generic_process(robot_head=self)

    def toggle_ros2_vr_connection(self) -> None:
        if self.ros2_vr_connection_status == 'inactive':
            self.activate_ros2()
        elif self.ros2_vr_connection_status == 'active':
            self.deactivate_ros2()
        else:
            print(f'ROS2 is in "{self.ros2_vr_connection_status}" state. Cannot be changed now')

    def toggle_lidar_listener(self) -> None:
        if self.lidar_listener_status == 'active' or self.lidar_listener_status == 'inactive':
            self.lidar_listener_status = 'processing'
        else:
            print(f'Lidar listener is in "{self.lidar_listener_status}" state. Cannot be changed now')

    def toggle_headlight(self) -> None:
        """Toggle the external RC LED light bar on/off. Entry point for the mobile app and controllers."""
        if self.headlight is None:
            if self.verbose >= 1:
                print('Headlight not available.')
            return
        self.headlight.toggle()
        if self.verbose >= 1:
            print(f'Headlight turned {"on" if self.headlight.is_on else "off"}')

    def turn_on_headlight(self) -> None:
        """Turn the external RC LED light bar on. Entry point for the mobile app."""
        if self.headlight is None:
            if self.verbose >= 1:
                print('Headlight not available.')
            return
        self.headlight.turn_on()

    def turn_off_headlight(self) -> None:
        """Turn the external RC LED light bar off. Entry point for the mobile app."""
        if self.headlight is None:
            if self.verbose >= 1:
                print('Headlight not available.')
            return
        self.headlight.turn_off()

    def add_mode_callback(self, mode: str, callback: callable, start: bool) -> None:
        assert mode in self.robot_mode_list, (f'Mode "{mode}" is not in the list of available'
                                              f' modes {self.robot_mode_list}')
        if start:
            if mode not in self.mode_start_callbacks:
                self.mode_start_callbacks[mode] = []
            self.mode_start_callbacks[mode].append(callback)
        else:
            if mode not in self.mode_end_callbacks:
                self.mode_end_callbacks[mode] = []
            self.mode_end_callbacks[mode].append(callback)
        if self.verbose >= 3:
            print(f'Added callback {callback} to {mode} mode {"start" if start else "end"} callbacks')

    def add_sub_mode_callback(self, sub_mode: str, callback: callable, start: bool) -> None:
        assert sub_mode in self.all_sub_modes(), (f'Sub mode "{sub_mode}" is not in the list of available'
                                                  f' sub modes {self.all_sub_modes()}')
        if start:
            if sub_mode not in self.sub_mode_start_callbacks:
                self.sub_mode_start_callbacks[sub_mode] = []
            self.sub_mode_start_callbacks[sub_mode].append(callback)
        else:
            if sub_mode not in self.sub_mode_end_callbacks:
                self.sub_mode_end_callbacks[sub_mode] = []
            self.sub_mode_end_callbacks[sub_mode].append(callback)
        if self.verbose >= 3:
            print(f'Added callback {callback} to {sub_mode} sub mode {"start" if start else "end"} callbacks')

    def all_sub_modes(self) -> list:
        """Returns a list of all sub modes available across all modes."""
        all_sub_modes = []
        for sub_modes in self.robot_sub_mode_dict.values():
            all_sub_modes.extend(sub_modes)
        return all_sub_modes

    def toggle_gui_mode(self) -> None:
        """
        Change between Graphical User Interface (GUI) mode and console mode.
        Note that Gnome terminal requires the GUI to work, so it can't be used in console mode.
        """
        self.gui_mode = not self.gui_mode
        if self.gui_mode:
            os.system(f'{gc.SCRIPT_FOLDER_PATH}start_gui.sh')
        else:
            print('stopping GUI mode')
            os.system(f'{gc.SCRIPT_FOLDER_PATH}stop_gui.sh')

    def set_movement(self, speed_x: float = None, speed_y: float = None, speed_z: float = None) -> None:
        """
        Set the wheel speeds of the robot. Leaves the speed unchanged if the parameter is None.
        :param speed_x: Speed in the X direction (forward/backward).
        :param speed_y: Speed in the Y direction (translate left/right).
        :param speed_z: Speed in the Z direction (rotate left/right).
        """
        # the speed values cannot all be None, at least one of them must be set
        assert speed_x is not None or speed_y is not None or speed_z is not None, \
            'At least one of the speed parameters must be set.'
        # if the speed is changed by any other method that set_movement_with_duration, remove the programmed stop
        self.stop_timestamp = None
        if speed_x is not None:
            self.speed_x = speed_x * self.speed_coefficient
        if speed_y is not None:
            self.speed_y = speed_y * self.speed_coefficient
        if speed_z is not None:
            self.speed_z = speed_z * self.speed_coefficient * self.steer_speed_proportion

    def set_movement_with_duration(self,
                                   duration: float,
                                   speed_x: float = None,
                                   speed_y: float = None,
                                   speed_z: float = None,
                                   ) -> None:
        """
        Set the wheel speeds of the robot and optionally set a duration for which the speed should be maintained.
        After the duration, the speed will be set to 0.
        :param speed_x: Speed in the X direction (forward/backward).
        :param speed_y: Speed in the Y direction (translate left/right).
        :param speed_z: Speed in the Z direction (rotate left/right).
        :param duration: Duration in seconds for which the speed should be maintained. duration must be between 0.1 and
         5 seconds.
        """
        if duration < 0.1 or duration > 5:
            warnings.warn(f'Duration must be between 0.1 and 5 seconds. Given: {duration} seconds.')
            duration = min(max(duration, 0.1), 5)
        # inverting values because this function is used by the voice interaction and not the joystick (which already
        # inverts the values)
        if speed_y is not None:
            speed_y = -speed_y
        if speed_z is not None:
            speed_z = -speed_z
        self.set_movement(speed_x=speed_x, speed_y=speed_y, speed_z=speed_z)
        self.stop_timestamp = time.time() + duration

    def check_programmed_stop(self) -> None:
        """
        Check if the programmed stop time has been reached. If so, set the speed to 0.
        This method should be called periodically to ensure the robot stops after the specified duration.
        """
        if self.stop_timestamp is not None and time.time() >= self.stop_timestamp:
            self.set_movement(speed_x=0, speed_y=0, speed_z=0)
            self.stop_timestamp = None
            # if self.verbose >= 3:
            #     print('Programmed stop reached, stopping the robot')
