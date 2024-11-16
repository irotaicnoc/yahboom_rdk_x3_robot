import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from SunriseRobotLib import SunriseRobot

from controller_sub import extra_args


class ControllerSubscriberNode(Node):
	def __init__(self, topic_name: str, queue_size: int):
		super().__init__('controller_subscriber_node')

		# subscriber
		self.get_logger().info(f'{topic_name=}')
		# self.get_logger().info(f'{queue_size=}')
		self.topic_name = topic_name
		self.queue_size = queue_size
		self.subscription = self.create_subscription(
			Twist,
			self.topic_name,
			self.listener_callback_function,
			self.queue_size,
		)
		self.car = SunriseRobot()

	def listener_callback_function(self, ros2_control_message: Twist):
		x_linear_speed = ros2_control_message.linear.x
		y_linear_speed = ros2_control_message.linear.y
		angular_speed = ros2_control_message.angular.z
		self.car.set_car_motion(x_linear_speed, y_linear_speed, angular_speed)
		

def main(args=None):
    print(f'{args=}')
    kwargs = extra_args.read()
    print(f'{kwargs=}')
    try:
        rclpy_init_args = kwargs['rclpy_init']
    except:
        rclpy_init_args = None
    rclpy.init(args=rclpy_init_args)
    try:
        controller_subscriber_node_args = kwargs['controller_subscriber_node']
    except:
        controller_subscriber_node_args = None
    controller_subscriber_node = ControllerSubscriberNode(**controller_subscriber_node_args)

    rclpy.spin(controller_subscriber_node)

    # close
    controller_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

