import os
import cv2
import time
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from hobot_vio import libsrcampy as srcampy

from camera_pub import extra_args


def format_camera_frames(frame, width: int, height: int, image_orientation: int, logger=None):
    frame_from_buffer = np.frombuffer(frame, dtype=np.uint8)
    # logger().info(f'frame_from_buffer shape: {frame_from_buffer.shape}')
    frame_reshaped = frame_from_buffer.reshape(height * 3 // 2, width)
    # logger().info(f'frame_reshaped shape: {frame_reshaped.shape}')
    frame_rgb = cv2.cvtColor(src=frame_reshaped, code=cv2.COLOR_YUV2RGB_NV12)
    # logger().info(f'frame_rgb shape: {frame_rgb.shape}')
    # logger().info(f'image_orientation: {image_orientation}')
    # logger().info(f'image_orientation type: {type(image_orientation)}')
    flipped_frame = cv2.flip(src=frame_rgb, flipCode=image_orientation)
    # logger().info(f'flipped frame shape: {flipped_frame.shape}')
    return flipped_frame


# def sensor_reset_shell():
#     os.system('echo 19 > /sys/class/gpio/export')
#     os.system('echo out > /sys/class/gpio/direction')
#     os.system('echo 0 > /sys/class/gpio/gpio19/value')
#     time.sleep(0.2)
#     os.system('echo 1 > /sys/class/gpio/gpio19/value')
#     os.system('echo 19 > /sys/class/gpio/unexport')
#     os.system('echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart')


class CameraPublisherNode(Node):
    def __init__(self,
                 topic_name: str,
                 image_orientation: int,
                 queue_size: int,
                 video_capture_kwargs: dict,
                 ):
        super().__init__('camera_publisher_node')
        # camera
        self.is_open = -1
        self.get_logger().info(f'{video_capture_kwargs=}')
        self.get_logger().info(f'{topic_name=}')
        # self.get_logger().info(f'{queue_size=}')
        self.video_capture_kwargs = video_capture_kwargs
        self.camera = srcampy.Camera()
        self.is_open = self.camera.open_cam(**self.video_capture_kwargs)
        self.get_logger().info(f'camera is_open = {self.is_open}')
        self.get_logger().info('0 = success, -1 = failed')
        self.image_width = self.video_capture_kwargs['width']
        self.image_height = self.video_capture_kwargs['height']
        self.image_orientation = image_orientation
        if self.is_open != 0:
            self.destroy_node()

        # convert images from cv2 to ros message format
        self.cv_ros_bridge = CvBridge()

        # publisher
        self.topic_name = topic_name
        self.queue_size = queue_size
        self.publisher = self.create_publisher(Image, self.topic_name, self.queue_size)

        # it is the inverse of Frames Per Second
        self.time_between_frames = round(1 / self.video_capture_kwargs['fps'], ndigits=3)
        self.timer = self.create_timer(self.time_between_frames, self.timer_callback_function)
        # self.message_counter = 0

    def timer_callback_function(self):
        frame = self.camera.get_img(2)
        if frame is None:
            self.get_logger().info('frame is None')
            return

        frame = format_camera_frames(
            frame=frame,
            width=self.image_width,
            height=self.image_height,
            image_orientation=self.image_orientation,
            # logger=self.get_logger,
        )
        # self.get_logger().info(f'time_between_frames {self.time_between_frames}')
        # self.get_logger().info(f'image_width {self.image_width}')
        # self.get_logger().info(f'image_height {self.image_height}')
        # ros2_image_message = self.cv_bridge.cv2_to_compressed_imgmsg(img, dst_format='jpeg')

        ros2_image_message = self.cv_ros_bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        # self.get_logger().info(f'image {self.message_counter}, shape {frame.shape}')
        # self.get_logger().info(f'encoding {ros2_image_message.encoding}')
        self.publisher.publish(ros2_image_message)

        # if self.message_counter % 100 == 0:
        #         self.get_logger().info(f'image {self.message_counter}, shape {frame.shape}')
        #         self.get_logger().info(f'encoding {ros2_image_message.encoding}')
        # self.message_counter += 1

    def destroy_node(self):
        self.get_logger().info('destroy cam publisher node')
        if self.is_open == 0:
            self.is_open = -1
            self.camera.close_cam()
        super().destroy_node()


def main(args=None):
    kwargs = extra_args.read()
    try:
        rclpy_init_args = kwargs['rclpy_init']
    except:
        rclpy_init_args = None
    rclpy.init(args=rclpy_init_args)
    try:
        camera_publisher_node_args = kwargs['camera_publisher_node']
    except:
        camera_publisher_node_args = None
    camera_publisher_node = CameraPublisherNode(**camera_publisher_node_args)

    rclpy.spin(camera_publisher_node)

    # close
    camera_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
