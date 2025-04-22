import os
import cv2
import time
import psutil
import numpy as np
from ultralytics import YOLO

import global_constants as gc


def format_camera_frames(frame,
                         original_width: int,
                         original_height: int,
                         # new_size: tuple = None,
                         ):
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
    # if new_size is not None:
    #     print(f'{new_size=}')
    #     frame_rgb = cv2.resize(frame_rgb, dsize=new_size)
    #     print(f'frame resized shape: {frame_rgb.shape}')
    return frame_rgb

    # Convert to JPEG
    # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
    # _, jpeg_buffer = cv2.imencode('.jpg', frame_rgb, encode_param)
    # jpeg_image = cv2.imdecode(jpeg_buffer, cv2.IMREAD_COLOR)
    # return jpeg_image


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


def change_range(value, original_min, original_max, new_min, new_max):
    return (value - original_min) * (new_max - new_min) / (original_max - original_min) + new_min


def x_displacement_to_angular_speed(x_distance_from_img_center: float,
                                    steer_threshold: float,
                                    angular_speed_range: list,
                                    ) -> float:
    # x_distance_from_img_center: [-1, -steer_threshold] [steer_threshold, 1]
    # output: [-2, -1] [1, 2]
    speed_z = change_range(
        value=abs(x_distance_from_img_center),
        original_min=steer_threshold,
        original_max=1,
        new_min=angular_speed_range[0],
        new_max=angular_speed_range[1],
    )
    if x_distance_from_img_center < 0:
        speed_z *= -1
    return speed_z


def sound_angle_to_robot_speed(sound_angle: float,
                               turn_only_angle: float,
                               forward_speed_range: list,
                               angular_speed_range: list,
                               ) -> (float, float):
    # sound_angle: [-turn_only_angle, turn_only_angle]
    # speed_x: [0, 0.6] forward-stationary (no backward movement)
    # speed_z: [-3, 3] left-right
    speed_x = change_range(
        value=-abs(sound_angle),
        original_min=-turn_only_angle,
        original_max=0,
        new_min=forward_speed_range[0],
        new_max=forward_speed_range[1],
    )
    speed_x = abs(speed_x)

    speed_z = change_range(
        value=abs(sound_angle),
        original_min=0,
        original_max=turn_only_angle,
        new_min=angular_speed_range[0],
        new_max=angular_speed_range[1],
    )
    if sound_angle < 0:
        speed_z *= -1
    return speed_x, speed_z


def display_image(image: np.ndarray,
                  proportion: float = 1.0,
                  window_name: str = 'Display image',
                  ) -> None:
    temp_image = image.copy()  # Make a copy of the original image

    if proportion != 1.0:
        height, width = temp_image.shape[:2]
        new_width = int(width * proportion)
        new_height = int(height * proportion)
        temp_image = cv2.resize(temp_image, (new_width, new_height))

    # Display the image
    cv2.imshow(window_name, temp_image)
    # Wait for a key press
    key = cv2.waitKey(0)
    cv2.destroyAllWindows()
    # If 'q' is pressed, exit
    if key == ord('q'):
        exit()


def voltage_to_percent(voltage: float) -> float:
    return change_range(value=voltage,
                        original_min=gc.MIN_VOLTAGE,
                        original_max=gc.MAX_VOLTAGE,
                        new_min=0.0,
                        new_max=100.0,
                        )


def microphone_angle_to_robot_angle(direction_of_arrival: float, microphone_robot_angle: float) -> float:
    assert 0 <= direction_of_arrival < 360, f'Invalid DOA angle: {direction_of_arrival}°'
    assert 0 <= microphone_robot_angle < 360, f'Invalid microphone forward angle: {microphone_robot_angle}°'

    # The DOA angle is the angle of the sound source relative to the microphone array.
    # The microphone array is mounted on the robot with a rotation of microphone_robot_angle°.

    converted_doa = direction_of_arrival + microphone_robot_angle
    while converted_doa >= 180:
        converted_doa -= 360
    assert -180 <= converted_doa < 180, f'Error in DOA conversion: {converted_doa}°'

    return converted_doa


def kill_process_(process_name: str, verbose: int = 0):
    target_found = True
    while target_found:
        target_found = False
        process_list = psutil.process_iter()
        if verbose >= 2:
            print(f'Killing process "{process_name}"...')
        for process in process_list:
            if process_name in process.name():
                target_found = True
                if verbose >= 1:
                    print(f'\t\t{process.name()} is running')
                os.kill(process.pid, 9)
                if verbose >= 2:
                    print(f'\t\t{process.name()} killed')
                os.system('sleep 0.1')


def start_generic_process(robot_head, name: str = None):
    robot_head.gpio_led.set_color('orange')
    if name is not None and robot_head.verbose >= 1:
        print(f'{name}...', end='')


def finish_generic_process(robot_head):
    robot_head.gpio_led.set_color('green')
    buzzer_previous_state = robot_head.buzzer_is_active
    robot_head.buzzer_is_active = True
    if robot_head.verbose >= 1:
        print('Done.')
    time.sleep(0.5)
    robot_head.gpio_led.set_color('off')
    robot_head.buzzer_is_active = buzzer_previous_state


def calculate_robot_direction(speed_x: float, speed_y: float, speed_z: float) -> float:
    # Calculate the direction of the robot given speed_x, speed_y, speed_z
    # speed_x: forward-backward speed
    # speed_y: translate left-right speed
    # speed_z: rotate left-right speed
    angle_degrees = 0
    if speed_x == 0 and speed_y == 0:
        angle_degrees = 90
    elif speed_x == 0:
        if speed_y > 0:
            angle_degrees = 180
        else:
            angle_degrees = 0
    elif speed_y == 0:
        if speed_x > 0:
            angle_degrees = 90
        else:
            angle_degrees = 270
    else:
        angle_radian = np.arctan2(speed_y, speed_x)
        angle_degrees = np.degrees(angle_radian)
        angle_degrees = (angle_degrees + 90) % 360

    # rotate by speed_z with a coefficient. It is arbitrary
    angle_degrees += speed_z * 30
    return angle_degrees
