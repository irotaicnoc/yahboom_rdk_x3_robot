import os
import cv2
import numpy as np
from ultralytics import YOLO

from kill_process import kill_process_


def format_camera_frames(frame, width: int, height: int):
    # if save_img:
    #     with open(f'/root/sunriseRobot/app_SunriseRobot/output/{counter}_01_frame_raw.raw', 'wb') as f:
    #         f.write(frame)
    frame_from_buffer = np.frombuffer(frame, dtype=np.uint8)
    # if save_img:
    #     np.save(f'/sunriseRobot/app_SunriseRobot/output/{counter}_02_frame_from_buffer.npy', frame_from_buffer)
    # logger().info(f'frame_from_buffer shape: {frame_from_buffer.shape}')
    frame_reshaped = frame_from_buffer.reshape(height * 3 // 2, width)
    # if save_img:
    #     np.save(f'/sunriseRobot/app_SunriseRobot/output/{counter}_03_frame_reshaped.npy', frame_reshaped)
    # logger().info(f'frame_reshaped shape: {frame_reshaped.shape}')
    frame_rgb = cv2.cvtColor(src=frame_reshaped, code=cv2.COLOR_YUV2BGR_NV12)
    # if save_img:
    #     cv2.imwrite(f'/sunriseRobot/app_SunriseRobot/output/{counter}_04_frame_rgb.jpg', frame_rgb)
    # logger().info(f'frame_rgb shape: {frame_rgb.shape}')
    # if counter >= 4:
    #     exit()
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
        for key in data:
            for i in range(_level):
                print('\t', end='')
            print(f'{key}:')
            pretty_print_dict(data[key], _level=_level + 1)
    else:
        for _ in range(_level):
            print('\t', end='')
        print(data)


def activate_hotspot(verbose: bool = False):
    if verbose:
        print("Starting Hotspot...", end='')
    os.system("sleep 3")
    os.system("systemctl stop wpa_supplicant")
    os.system("ip addr flush dev wlan0")
    os.system("sleep 0.5")
    os.system("ifconfig wlan0 down")
    os.system("sleep 1")
    os.system("ifconfig wlan0 up")
    os.system("hostapd -B /root/sunriseRobot/hotspot/etc/hostapd.conf")
    os.system("ifconfig wlan0 192.168.8.88 netmask 255.255.255.0")
    os.system("systemctl start isc-dhcp-server")
    if verbose:
        print("Done.")


def deactivate_hotspot(verbose: bool = False):
    if verbose:
        print("Stopping Hotspot...", end='')
    kill_process_(program_name="hostapd", debug=verbose)
    os.system("systemctl stop isc-dhcp-server")
    os.system("ip addr flush dev wlan0")
    os.system("sleep 0.5")
    os.system("ifconfig wlan0 down")
    os.system("sleep 1")
    os.system("ifconfig wlan0 up")
    os.system("systemctl start wpa_supplicant")
    if verbose:
        print("Done.")


def activate_ros2(verbose: bool = False):
    if verbose:
        print("Starting ROS2...", end='')
    os.system("/root/sunriseRobot/app_SunriseRobot/start_ros2.sh")
    if verbose:
        print("Done.")


def deactivate_ros2(verbose: bool = False):
    # TODO: it does not really kill the process in the separate console
    if verbose:
        print("Stopping ROS2...", end='')
    kill_process_(program_name="ros2", debug=verbose)
    if verbose:
        print("Done.")


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
