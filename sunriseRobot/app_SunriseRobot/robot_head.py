import os
from pathlib import Path

import args
import utils
import global_constants as gc


class RobotHead:
    def __init__(self, **kwargs):
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'robot_head.yaml',
            **kwargs,
        )
        self.verbose = parameters['verbose']
        self.controller_id_list = []
        self.connected_controllers = 0

        # autonomous mode parameters
        # self.robot_mode_list = ['user_control_wheels', ]
        self.robot_mode_list = []
        self.robot_mode_list.append('user_control_wheels')
        self.robot_mode = self.robot_mode_list[0]
        if self.verbose >= 1:
            print(f'Robot mode: {self.robot_mode}')
        self.tracking_target_list = parameters['tracking_target_list']
        self.tracking_target_pos = 0
        # search for models in the model folder
        self.model_list = []
        self.model_pos = 0
        model_folder_path = Path(gc.GENERIC_MODEL_FOLDER_PATH)
        for model_path in model_folder_path.glob('*.*'):
            self.model_list.append(model_path.name)

        # hotspot and ROS2 parameters
        self.ros2_vr_connection_status = 'inactive'
        self.hotspot_status = 'inactive'
        self.hotspot_ip = parameters['hotspot_ip']

        # motion parameters
        self.steer_speed_proportion = parameters['steer_speed_proportion']
        self.speed_coefficient = parameters['speed_coefficient']
        # wheel speed
        self.speed_x = 0
        self.speed_y = 0
        self.speed_z = 0

        # buzzer, leds, and lights
        self.buzzer_is_active = False
        self.internal_light = parameters['internal_light']
        self.gpio_led = parameters['gpio_led']

        # arm parameters
        if parameters['arm_present']:
            self.robot_mode_list.append('user_control_arm')
            self.arm_speed_proportion = parameters['arm_speed_proportion']
            self.arm_is_rigid = True
            self.arm_state_not_updated = False
            # arm servos
            self.arm_speed = [0, 0, 0, 0, 0, 0]
            # servo angles have to be in the range [0, 180], except for servo 4 which has range [0, 270]
            # all servos to 90 degrees means vertical position
            # during each loop iteration, the desired angle is updated by adding the speed
            # and the real angle is moved closer to the desired angle
            self.arm_desired_angles = parameters['arm_initial_angles']
            if len(self.arm_desired_angles) > 6:
                if self.verbose >= 1:
                    print(f'The robot supports at most a 6-servos arm,'
                          f' but {len(self.arm_desired_angles)} were provided. ')
            # speed with which the arm reaches the desired angle [0, 2000]
            # 0 is the fastest speed, 2000 is the slowest speed
            # for manual control use 0, for arbitrary position specified directly via arm_desired_angles
            # use a slower speed (higher value)
            self.run_time = 0
            self.button_press_time = 0
            self.one_time_check = False

    def next_mode(self):
        if self.verbose >= 3:
            print(f'Switching from {self.robot_mode} mode.')
        self.robot_mode = self.robot_mode_list[
            (self.robot_mode_list.index(self.robot_mode) + 1) % len(self.robot_mode_list)
        ]
        self.gpio_led.set_color('off')
        self.internal_light.stop()

        if self.verbose >= 1:
            print(f'Switching to {self.robot_mode} mode.')

    def next_target(self):
        self.tracking_target_pos += 1
        self.tracking_target_pos = self.tracking_target_pos % len(self.tracking_target_list)
        if self.verbose >= 1:
            print(f'New target: {self.tracking_target_list[self.tracking_target_pos]}')

    def previous_target(self):
        self.tracking_target_pos -= 1
        self.tracking_target_pos = self.tracking_target_pos % len(self.tracking_target_list)
        if self.verbose >= 1:
            print(f'New target: {self.tracking_target_list[self.tracking_target_pos]}')

    def next_model(self):
        self.model_pos += 1
        self.model_pos = self.model_pos % len(self.model_list)
        if self.verbose >= 1:
            print(f'New model: {self.model_list[self.model_pos]}')

    def previous_model(self):
        self.model_pos -= 1
        self.model_pos = self.model_pos % len(self.model_list)
        if self.verbose >= 1:
            print(f'New model: {self.model_list[self.model_pos]}')

    def increase_speed_coefficient(self):
        self.speed_coefficient = min(1.0, self.speed_coefficient + 0.1)
        if self.verbose >= 2:
            print(f'Speed coefficient: {self.speed_coefficient}')

    def decrease_speed_coefficient(self):
        self.speed_coefficient = max(0.1, self.speed_coefficient - 0.1)
        if self.verbose >= 2:
            print(f'Speed coefficient: {self.speed_coefficient}')

    def toggle_arm_rigid(self):
        self.arm_is_rigid = not self.arm_is_rigid
        self.arm_state_not_updated = True
        if self.verbose >= 1:
            if self.arm_is_rigid:
                print(f'Arm is rigid')
            else:
                print(f'Arm can be moved manually, but cannot be controlled by the controller')

    def update_arm_speed(self, servo_id: int, value) -> None:
        self.run_time = 0
        self.arm_speed[servo_id] = value * self.speed_coefficient * self.arm_speed_proportion

    def update_arm_desired_angles(self) -> None:
        for servo_id in range(len(self.arm_speed)):
            servo_speed = self.arm_speed[servo_id]
            temp_angle = self.arm_desired_angles[servo_id] + servo_speed
            if temp_angle < 0:
                temp_angle = 0
            if temp_angle > 180:
                temp_angle = 180
            self.arm_desired_angles[servo_id] = temp_angle

    def set_arm_desired_angles(self, angle_list: list) -> None:
        assert len(angle_list) == len(self.arm_desired_angles), \
            (f'Length of angle_list {len(angle_list)} is not equal'
             f' to arm_servos_desired_angle {len(self.arm_desired_angles)}')

        self.run_time = utils.change_range(
            value=self.speed_coefficient,
            original_min=0.1,
            original_max=1,
            new_min=2000,
            new_max=400,
        )
        print(f'check run_time: {self.run_time}')
        self.arm_desired_angles = angle_list

    def activate_hotspot(self):
        if self.hotspot_status == 'active':
            if self.verbose >= 2:
                print('Hotspot is already active.')
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

    def deactivate_hotspot(self):
        if self.hotspot_status == 'inactive':
            if self.verbose >= 2:
                print('Hotspot is already inactive.')
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

    def toggle_hotspot(self):
        if self.hotspot_status == 'inactive':
            self.activate_hotspot()
        elif self.hotspot_status == 'active':
            self.deactivate_hotspot()
        else:
            print(f'Hotspot is in "{self.hotspot_status}" state. Cannot be changed now.')

    def activate_ros2_vr_connection(self):
        if self.ros2_vr_connection_status == 'active':
            if self.verbose >= 2:
                print('ROS2 VR connection is already active.')
            return
        self.ros2_vr_connection_status = 'processing'
        utils.start_generic_process(robot_head=self, name='Starting ROS2')
        # os.system(f'{gc.SCRIPT_FOLDER_PATH}start_ros2.sh')
        os.system('gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash;cd /root/marco_ros2_ws/;'
                  'source install/local_setup.bash;ros2 launch ros_tcp_endpoint endpoint_launch.py;exec bash"')
        self.hotspot_status = 'active'
        utils.finish_generic_process(robot_head=self)

    def deactivate_ros2(self):
        # TODO: it does not really kill the process in the separate console
        if self.ros2_vr_connection_status == 'inactive':
            if self.verbose >= 2:
                print('ROS2 is already inactive.')
            return
        self.ros2_vr_connection_status = 'processing'
        utils.start_generic_process(robot_head=self, name='Stopping ROS2')
        utils.kill_process_(process_name='ros2', verbose=self.verbose)
        self.gpio_led.set_color('green')
        self.ros2_vr_connection_status = 'inactive'
        utils.finish_generic_process(robot_head=self)

    def toggle_ros2_vr_connection(self):
        if self.ros2_vr_connection_status == 'inactive':
            self.activate_ros2_vr_connection()
        elif self.ros2_vr_connection_status == 'active':
            self.deactivate_ros2()
        else:
            print(f'ROS2 is in "{self.ros2_vr_connection_status}" state. Cannot be changed now.')
