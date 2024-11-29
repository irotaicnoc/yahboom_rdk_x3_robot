# standard libraries
import time

# ros2 libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8

# robot libraries
import smbus

# my libraries
from controller_sub import utils
from controller_sub.robot_body import RobotBody


class ControllerSubscriberNode(Node):
    def __init__(self,
                 motion_topic: str,
                 buzzer_topic: str,
                 rgb_light_topic: str,
                 queue_size: int,
                 verbose: int = 0,
                 ):
        super().__init__('controller_subscriber_node')

        self.verbose = verbose
        if self.verbose >= 1:
            self.get_logger().info(f'{motion_topic=}')
            self.get_logger().info(f'{buzzer_topic=}')
            self.get_logger().info(f'{rgb_light_topic=}')
            # self.get_logger().info(f'{queue_size=}')
        self.queue_size = queue_size
        self.robot = RobotBody()
        self.bus = smbus.SMBus(0)

        # motion subscriber
        self.motion_topic = motion_topic
        self.subscription = self.create_subscription(
            Twist,
            self.motion_topic,
            self.motion_listener_callback,
            self.queue_size,
        )

        # buzzer subscriber
        self.buzzer_topic = buzzer_topic
        self.subscription = self.create_subscription(
            Bool,
            self.buzzer_topic,
            self.buzzer_listener_callback,
            self.queue_size,
        )

        # rgb_light subscriber
        self.rgb_light_topic = rgb_light_topic
        self.subscription = self.create_subscription(
            Int8,
            self.rgb_light_topic,
            self.rgb_light_listener_callback,
            self.queue_size,
        )

    def motion_listener_callback(self, motion_message: Twist):
        if not isinstance(motion_message, Twist):
            return
        x_linear_speed = motion_message.linear.x
        y_linear_speed = motion_message.linear.y
        angular_speed = motion_message.angular.z
        self.robot.set_car_motion(x_linear_speed, y_linear_speed, angular_speed)

    def buzzer_listener_callback(self, buzzer_message: Bool):
        if not isinstance(buzzer_message, Bool):
            return
        if buzzer_message.data:
            self.robot.set_beep(1)
        else:
            self.robot.set_beep(0)

    def rgb_light_listener_callback(self, rgb_light_message: Int8):
        if not isinstance(rgb_light_message, Int8):
            return
        rgb_light_value = rgb_light_message.data
        # print (f'RGBLight: {rgb_light_value}')
        if rgb_light_value > 4:
            self.get_logger().warning(f'RGBLight data = {rgb_light_value} > 4')
            rgb_light_value = 0
        if rgb_light_value == 0:
            self.bus.write_byte_data(0x0d, 0x07, 0x00)
            time.sleep(.05)
        else:
            self.bus.write_byte_data(0x0d, 0x04, rgb_light_value)
            time.sleep(.05)


def main(args=None):
    rclpy.init(args=args)
    kwargs = utils.args_from_yaml(config_path='/root/marco_ros2_ws/src/controller_sub/controller_sub/config.yaml')
    controller_subscriber_node = ControllerSubscriberNode(**kwargs)

    rclpy.spin(controller_subscriber_node)

    # close
    controller_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
