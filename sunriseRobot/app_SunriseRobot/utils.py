import os
import cv2
import numpy as np
from ultralytics import YOLO

import global_constants as gc
from kill_process import kill_process_


def format_camera_frames(frame, original_width: int, original_height: int, new_size: tuple = None):
    # if save_img:
    #     with open(f'{gc.APP_FOLDER_PATH}output/{counter}_01_frame_raw.raw', 'wb') as f:
    #         f.write(frame)
    frame_from_buffer = np.frombuffer(frame, dtype=np.uint8)
    # if save_img:
    #     np.save(f'{gc.APP_FOLDER_PATH}output/{counter}_02_frame_from_buffer.npy', frame_from_buffer)
    # print(f'frame_from_buffer shape: {frame_from_buffer.shape}')
    frame_reshaped = frame_from_buffer.reshape(original_height * 3 // 2, original_width)
    # if save_img:
    #     np.save(f'{gc.APP_FOLDER_PATH}output/{counter}_03_frame_reshaped.npy', frame_reshaped)
    # print(f'frame_reshaped shape: {frame_reshaped.shape}')
    frame_rgb = cv2.cvtColor(src=frame_reshaped, code=cv2.COLOR_YUV2BGR_NV12)
    # if save_img:
    #     cv2.imwrite(f'{gc.APP_FOLDER_PATH}output/{counter}_04_frame_rgb.jpg', frame_rgb)
    # print(f'frame RGB shape: {frame_rgb.shape}')
    # if counter >= 4:
    #     exit()
    if new_size is not None:
        # print(f'{new_size=}')
        frame_rgb = cv2.resize(frame_rgb, dsize=new_size)
        # print(f'frame resized shape: {frame_rgb.shape}')
    return frame_rgb

# def sensor_reset_shell():
#    os.system('echo 19 > /sys/class/gpio/export')
#    os.system('echo out > /sys/class/gpio/direction')
#    os.system('echo 0 > /sys/class/gpio/gpio19/value')
#    time.sleep(0.2)
#    os.system('echo 1 > /sys/class/gpio/gpio19/value')
#    os.system('echo 19 > /sys/class/gpio/unexport')
#    os.system('echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart')


def get_class_id_from_name(class_name: str, class_dict: dict) -> int:
    for class_id, name in class_dict.items():
        if name == class_name:
            return class_id
    raise ValueError(f'Class name not found: {class_name}')


def print_known_classes(model=None, yolo_model_name: str = None) -> None:
    # exactly one of model or yolo_model_name must be provided
    assert model is None or yolo_model_name is None, 'Only one of "model" or "yolo_model_name" must be provided'
    assert model is not None or yolo_model_name is not None, 'Either "model" or "yolo_model_name" must be provided'

    if yolo_model_name is not None:
        model = YOLO(model=yolo_model_name, verbose=False)

    for class_id, class_name in enumerate(model.names):
        print(f'{model.names[class_id]}, ID: {class_id}')


def pretty_print_dict(data, _level: int = 0) -> None:
    if isinstance(data, dict):
        if _level > 0:
            print()
        for key in data:
            for i in range(_level + 1):
                print('\t', end='')
            print(f'{key}: ', end='')
            pretty_print_dict(data[key], _level=_level + 1)
    else:
        print(data)


def activate_hotspot(hotspot_ip: str, verbose: int = 0):
    if verbose >= 1:
        print('Starting Hotspot...', end='')
    os.system('sleep 2')
    os.system('systemctl stop wpa_supplicant')
    os.system('ip addr flush dev wlan0')
    os.system('sleep 0.5')
    os.system('ifconfig wlan0 down')
    os.system('sleep 1')
    os.system('ifconfig wlan0 up')
    os.system(f'hostapd -B {gc.MAIN_FOLDER_PATH}hotspot/etc/hostapd.conf')
    os.system(f'ifconfig wlan0 {hotspot_ip} netmask 255.255.255.0')
    os.system('systemctl start isc-dhcp-server')
    if verbose >= 1:
        print('Done.')


def deactivate_hotspot(verbose: int = 0):
    if verbose >= 1:
        print('Stopping Hotspot...', end='')
    kill_process_(process_name='hostapd', verbose=verbose)
    os.system('systemctl stop isc-dhcp-server')
    os.system('ip addr flush dev wlan0')
    os.system('sleep 0.5')
    os.system('ifconfig wlan0 down')
    os.system('sleep 1')
    os.system('ifconfig wlan0 up')
    os.system('systemctl start wpa_supplicant')
    if verbose >= 1:
        print('Done.')


def activate_ros2(verbose: int = 0):
    if verbose >= 1:
        print('Starting ROS2...', end='')
    # os.system(f'{gc.APP_FOLDER_PATH}start_ros2.sh')
    os.system('gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash;cd /root/marco_ros2_ws/;'
              'source install/local_setup.bash;ros2 launch ros_tcp_endpoint endpoint_launch.py;exec bash"')
    # os.system('wait')
    # os.system('exit 0')
    if verbose >= 1:
        print('Done.')


def deactivate_ros2(verbose: int = 0):
    # TODO: it does not really kill the process in the separate console
    if verbose >= 1:
        print('Stopping ROS2...', end='')
    kill_process_(process_name='ros2', verbose=verbose)
    if verbose >= 1:
        print('Done.')


def change_range(val, original_min_val, original_max_val, new_min_val, new_max_val):
    return (val - original_min_val) * (new_max_val - new_min_val) / (original_max_val - original_min_val) + new_min_val


def x_displacement_to_angular_speed(x_distance_from_img_center: float,
                                    steer_threshold: float,
                                    angular_speed_range: list,
                                    ) -> float:
    # x_distance_from_img_center: [-1, -steer_threshold] [steer_threshold, 1]
    # output: [-2, -1] [1, 2]
    speed_z = change_range(
        val=abs(x_distance_from_img_center),
        original_min_val=steer_threshold,
        original_max_val=1,
        new_min_val=angular_speed_range[0],
        new_max_val=angular_speed_range[1],
    )
    if x_distance_from_img_center < 0:
        speed_z *= -1
    return speed_z
