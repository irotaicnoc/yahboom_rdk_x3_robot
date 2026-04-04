# ros2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# robot libraries
from hobot_vio import libsrcampy as camera_lib

# my libraries
from camera_pub import utils


class CameraPublisherNode(Node):
    def __init__(self,
                 camera_topic: str,
                 queue_size: int,
                 video_capture_kwargs: dict,
                 verbose: int = 0,
                 ):
        super().__init__('camera_publisher_node')
        # camera
        self.is_open = -1
        self.verbose = verbose
        if self.verbose >= 1:
            self.get_logger().info(f'{video_capture_kwargs=}')
            self.get_logger().info(f'{camera_topic=}')
            # self.get_logger().info(f'{queue_size=}')
        self.video_capture_kwargs = video_capture_kwargs
        self.camera = camera_lib.Camera()
        self.is_open = self.camera.open_cam(**self.video_capture_kwargs)
        if self.is_open == 0:
            self.get_logger().info(f'camera is_open: SUCCESS')
        else:
            self.get_logger().info(f'camera is_open: FAILED')
        self.image_width = self.video_capture_kwargs['width']
        self.image_height = self.video_capture_kwargs['height']
        if self.is_open != 0:
            self.destroy_node()

        # publisher
        self.topic_name = camera_topic
        self.queue_size = queue_size
        self.publisher = self.create_publisher(CompressedImage, self.topic_name, self.queue_size)

        # it is the inverse of Frames Per Second
        self.time_between_frames = round(1 / self.video_capture_kwargs['fps'], ndigits=3)
        self.timer = self.create_timer(self.time_between_frames, self.timer_callback_function)
        self.message_counter = 0

    def timer_callback_function(self):
        frame = self.camera.get_img(2)
        if frame is None:
            self.get_logger().info('Frame is None')
            return
        # save_img = False
        # if self.message_counter % 100 == 0:
        #     save_img = True
        frame = utils.format_camera_frames(
            frame=frame,
            width=self.image_width,
            height=self.image_height,
            # logger=self.get_logger,
            # save_img=save_img,
            # counter=self.message_counter//100,
        )
        # self.get_logger().info(f'time_between_frames {self.time_between_frames}')
        # self.get_logger().info(f'image_width {self.image_width}')
        # self.get_logger().info(f'image_height {self.image_height}')

        current_time = self.get_clock().now()
        ros2_image_message = utils.jpeg_to_compressed_img_msg(frame, timestamp=current_time)
        self.publisher.publish(ros2_image_message)

        # if self.message_counter % 100 == 0:
        #         self.get_logger().info(f'image {self.message_counter}, shape {frame.shape}')
        #         self.get_logger().info(f'encoding {ros2_image_message.encoding}')
        self.message_counter += 1

    def destroy_node(self):
        self.get_logger().info('Destroying cam publisher node')
        if self.is_open == 0:
            self.is_open = -1
            self.camera.close_cam()
        super().destroy_node()


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    kwargs = utils.args_from_yaml(config_path='/root/marco_ros2_ws/src/camera_pub/camera_pub/config.yaml')
    camera_publisher_node = CameraPublisherNode(**kwargs)

    rclpy.spin(camera_publisher_node)

    # close
    camera_publisher_node.destroy_node()


if __name__ == '__main__':
    main()
