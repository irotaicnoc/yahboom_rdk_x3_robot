# ros2 libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# robot libraries
from hobot_vio import libsrcampy as camera_lib

# my libraries
from camera_pub import utils


class CameraPublisherNode(Node):
    def __init__(self,
                 camera_topic: str,
                 image_orientation: int,
                 queue_size: int,
                 video_capture_kwargs: dict,
                 ):
        super().__init__('camera_publisher_node')
        # camera
        self.is_open = -1
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
        self.image_orientation = image_orientation
        if self.is_open != 0:
            self.destroy_node()

        # convert images from cv2 to ros message format
        self.cv_ros_bridge = CvBridge()

        # publisher
        self.topic_name = camera_topic
        self.queue_size = queue_size
        self.publisher = self.create_publisher(Image, self.topic_name, self.queue_size)

        # it is the inverse of Frames Per Second
        self.time_between_frames = round(1 / self.video_capture_kwargs['fps'], ndigits=3)
        self.timer = self.create_timer(self.time_between_frames, self.timer_callback_function)
        self.message_counter = 0

    def timer_callback_function(self):
        frame = self.camera.get_img(2)
        if frame is None:
            self.get_logger().info('frame is None')
            return
        # save_img = False
        # if self.message_counter % 100 == 0:
        #     save_img = True
        frame = utils.format_camera_frames(
            frame=frame,
            width=self.image_width,
            height=self.image_height,
            image_orientation=self.image_orientation,
            # logger=self.get_logger,
            # save_img=save_img,
            # counter=self.message_counter//100,
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
        self.message_counter += 1

    def destroy_node(self):
        self.get_logger().info('destroy cam publisher node')
        if self.is_open == 0:
            self.is_open = -1
            self.camera.close_cam()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    kwargs = utils.args_from_yaml(config_path='/root/GIT/yahboom_rdk_x3_robot/src/camera_pub/camera_pub/config.yaml')
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
