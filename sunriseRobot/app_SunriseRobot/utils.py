import cv2
import numpy as np
from ultralytics import YOLO


def format_camera_frames(frame, width: int, height: int):
    # if save_img:
    #     with open(f'/root/marco_ros2_ws/src/camera_pub/camera_pub/{counter}_01_frame_raw.raw', 'wb') as f:
    #         f.write(frame)
    frame_from_buffer = np.frombuffer(frame, dtype=np.uint8)
    # if save_img:
    #     np.save(f'/root/marco_ros2_ws/src/camera_pub/camera_pub/{counter}_02_frame_from_buffer.npy', frame_from_buffer)
    # logger().info(f'frame_from_buffer shape: {frame_from_buffer.shape}')
    frame_reshaped = frame_from_buffer.reshape(height * 3 // 2, width)
    # if save_img:
    #     np.save(f'/root/marco_ros2_ws/src/camera_pub/camera_pub/{counter}_03_frame_reshaped.npy', frame_reshaped)
    # logger().info(f'frame_reshaped shape: {frame_reshaped.shape}')
    frame_rgb = cv2.cvtColor(src=frame_reshaped, code=cv2.COLOR_YUV2RGB_NV12)
    # if save_img:
    #     cv2.imwrite(f'/root/marco_ros2_ws/src/camera_pub/camera_pub/{counter}_04_frame_rgb.jpg', frame_rgb)
    # logger().info(f'frame_rgb shape: {frame_rgb.shape}')
    # logger().info(f'image_orientation: {image_orientation}')
    # logger().info(f'image_orientation type: {type(image_orientation)}')
    # flipped_frame = cv2.flip(src=frame_rgb, flipCode=image_orientation)
    # if save_img:
    #     cv2.imwrite(f'/root/marco_ros2_ws/src/camera_pub/camera_pub/{counter}_05_flipped_frame.jpg', flipped_frame)
    # logger().info(f'flipped frame shape: {flipped_frame.shape}')
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


def pretty_print_dict(data, _level: int = 0):
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