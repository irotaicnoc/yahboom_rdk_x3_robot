import os
import time
import warnings
from pathlib import Path

import args
import utils
import global_constants as gc


class RobotHead:
    def __init__(self, **kwargs):
        self.robot_body = kwargs['robot_body']
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'robot_head.yaml', **kwargs)
        self.gui_mode = parameters['gui_mode']
        self.verbose = parameters['verbose']

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
        self.internal_light = parameters['internal_light']
        self.led_3_pin = parameters['led_3_pin']

    def next_mode(self) -> None:
        if self.verbose >= 3:
            print(f'Switching from {self.robot_mode} ({self.robot_sub_mode}) mode')
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
        # if the previous mode has callbacks to call at the end, call them. But only if the mode has actually changed.
        # For example if the list has only 1 element, the callback should not be called, because the robot was already
        # in the same mode. Or if the new mode fails to be set and the previous mode is set again
        if previous_mode != self.robot_mode:
            if previous_mode in self.mode_end_callbacks:
                for callback in self.mode_end_callbacks[previous_mode]:
                    callback()
            # if the new mode has callbacks to call at the start, call them. But only if the mode has actually changed
            if self.robot_mode in self.mode_start_callbacks:
                for callback in self.mode_start_callbacks[self.robot_mode]:
                    callback()

        # sub mode callbacks
        # if the sub mode also have callbacks to call at the start and end, call them. But only if the mode current
        # mode has a sub mode, and the sub mode actually changed
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
            # if the sub mode also have callbacks to call at the start and end, call them. But only if the sub mode has
            # actually changed. For example if the list has only 1 element, the callbacks should not be called, because
            # the robot was already in the same sub mode. Or if the new sub mode fails to be set and the previous sub
            # mode is set again.
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
            os.system('gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash;cd /root/marco_ros2_ws/;'
                      'source install/local_setup.bash;ros2 launch ros_tcp_endpoint endpoint_launch.py;exec bash"')
        else:
            os.system(f'{gc.SCRIPT_FOLDER_PATH}start_ros2_no_gui.sh')
        self.ros2_vr_connection_status = 'active'
        utils.finish_generic_process(robot_head=self)

    def deactivate_ros2(self) -> None:
        # TODO: it does not really kill the process in the separate console
        if self.ros2_vr_connection_status == 'inactive':
            if self.verbose >= 2:
                print('ROS2 is already inactive')
            return
        self.ros2_vr_connection_status = 'processing'
        utils.start_generic_process(robot_head=self, name='Stopping ROS2')
        utils.kill_process_(process_name='ros2', verbose=self.verbose)
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
        if self.lidar_listener_status == 'inactive':
            self.lidar_listener_status = 'processing'
        elif self.lidar_listener_status == 'active':
            self.lidar_listener_status = 'processing'
        else:
            print(f'Lidar listener is in "{self.lidar_listener_status}" state. Cannot be changed now')

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
        self.set_movement(speed_x=speed_x, speed_y=-speed_y, speed_z=-speed_z)
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
