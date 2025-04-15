# ros2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# my libraries
from lidar_sub import utils


class LidarSubscriberNode(Node):
    def __init__(self,
                 lidar_topic: str,
                 queue_size: int,
                 verbose: int = 0,
                 ):
        super().__init__('lidar_subscriber_node')

        self.verbose = verbose
        if self.verbose >= 1:
            self.get_logger().info(f'{lidar_topic=}')
        self.queue_size = queue_size

        self.lidar_topic = lidar_topic
        self.subscription = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.lidar_listener_callback,
            self.queue_size,
        )

    def lidar_listener_callback(self, lidar_message: LaserScan):
        if not isinstance(lidar_message, LaserScan):
            return
        if self.verbose >= 1:
            self.get_logger().info(f'{lidar_message=}')


def main(args=None):
    rclpy.init(args=args)
    kwargs = utils.args_from_yaml(config_path='/root/marco_ros2_ws/src/lidar_sub/lidar_sub/config.yaml')
    lidar_subscriber_node = LidarSubscriberNode(**kwargs)

    rclpy.spin(lidar_subscriber_node)

    # close
    lidar_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
