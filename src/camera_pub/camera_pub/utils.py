# standard libraries
# import os
# import time
import cv2
import yaml
import numpy as np
from sensor_msgs.msg import CompressedImage


def args_from_yaml(config_path: str) -> dict:
    with open(config_path) as config_file:
        arg_dict = yaml.safe_load(config_file)
    return arg_dict


def format_camera_frames(frame, width: int, height: int):
    frame = np.frombuffer(frame, dtype=np.uint8)
    # logger().info(f'frame from buffer shape: {frame.shape}')
    frame = frame.reshape(height * 3 // 2, width)
    # logger().info(f'frame reshaped shape: {frame.shape}')
    # frame = cv2.cvtColor(src=frame, code=cv2.COLOR_YUV2RGB_NV12)
    frame = cv2.cvtColor(src=frame, code=cv2.COLOR_YUV2BGR_NV12)
    # logger().info(f'frame rgb shape: {frame.shape}')
    # flip image horizontally (left and right are inverted for some reason)
    # frame = cv2.flip(src=frame, flipCode=1)
    # Flip the image vertically (upside down)
    # logger().info(f'flipped frame shape: {frame.shape}')
    return frame


# def sensor_reset_shell():
#    os.system('echo 19 > /sys/class/gpio/export')
#    os.system('echo out > /sys/class/gpio/direction')
#    os.system('echo 0 > /sys/class/gpio/gpio19/value')
#    time.sleep(0.2)
#    os.system('echo 1 > /sys/class/gpio/gpio19/value')
#    os.system('echo 19 > /sys/class/gpio/unexport')
#    os.system('echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart')


def jpeg_to_compressed_img_msg(frame, timestamp):
    """
    Converts an OpenCV image to a ROS image without using the cv_bridge package,
    for compatibility purposes.
    """
    # Convert to JPEG
    encode_param = [cv2.IMWRITE_JPEG_QUALITY, 80]
    _, jpeg_buffer = cv2.imencode('.jpg', frame, encode_param)
    # convert the numpy array to a bytes object, which is what the ROS message expects
    jpeg_buffer = jpeg_buffer.tobytes()
    msg = CompressedImage()
    msg.header.stamp = timestamp.to_msg()
    msg.format = 'jpeg'
    msg.data = jpeg_buffer

    return msg
