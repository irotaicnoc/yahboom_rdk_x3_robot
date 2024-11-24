# standard libraries
# import os
# import time
import cv2
import yaml
import numpy as np


def args_from_yaml(config_path: str) -> dict:
    with open(config_path) as config_file:
        arg_dict = yaml.safe_load(config_file)
    return arg_dict


def format_camera_frames(frame, width: int, height: int, image_orientation: int):
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
    # frame_rgb = cv2.cvtColor(src=frame_reshaped, code=cv2.COLOR_YUV2BGR_NV12)
    # if save_img:
    #     cv2.imwrite(f'/root/marco_ros2_ws/src/camera_pub/camera_pub/{counter}_04_frame_rgb.jpg', frame_rgb)
    # logger().info(f'frame_rgb shape: {frame_rgb.shape}')
    # logger().info(f'image_orientation: {image_orientation}')
    # logger().info(f'image_orientation type: {type(image_orientation)}')
    flipped_frame = cv2.flip(src=frame_rgb, flipCode=image_orientation)
    # if save_img:
    #     cv2.imwrite(f'/root/marco_ros2_ws/src/camera_pub/camera_pub/{counter}_05_flipped_frame.jpg', flipped_frame)
    # logger().info(f'flipped frame shape: {flipped_frame.shape}')
    # if counter >= 4:
    #     exit()
    return flipped_frame

# def sensor_reset_shell():
#    os.system('echo 19 > /sys/class/gpio/export')
#    os.system('echo out > /sys/class/gpio/direction')
#    os.system('echo 0 > /sys/class/gpio/gpio19/value')
#    time.sleep(0.2)
#    os.system('echo 1 > /sys/class/gpio/gpio19/value')
#    os.system('echo 19 > /sys/class/gpio/unexport')
#    os.system('echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart')
