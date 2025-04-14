import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import threading


class LidarListener(Node):
    def __init__(self, topic_name: str, queue_size=10):
        super().__init__('lidar_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            topic_name,
            self.scan_callback,
            queue_size
        )

        self.scan_data = None  # Store the latest processed data
        self.already_seen = False

    def scan_callback(self, msg: LaserScan) -> None:
        # self.get_logger().info('Published processed lidar data')
        self.scan_data = self.process_scan_data(msg)
        self.already_seen = False

    def get_latest_scan_data(self) -> (LaserScan, bool):
        old_seen_value = self.already_seen
        self.already_seen = True
        return self.scan_data, old_seen_value


def spin_node(node: LidarListener) -> None:
    rclpy.spin(node)


def main(topic_name: str, queue_size: int) -> None:
    rclpy.init()
    listener_node = LidarListener(topic_name=topic_name, queue_size=queue_size)

    # Spin the node in a separate thread so it doesn't block the main program
    spin_thread = threading.Thread(target=spin_node, args=(listener_node,))
    spin_thread.start()

    try:
        while True:
            # Access the latest processed data from the node
            latest_data = listener_node.get_latest_scan_data()
            if latest_data is not None:
                print(f"Main program received processed data: {latest_data}")
            # Perform other tasks in your main program
            import time
            time.sleep(1)  # Example: Sleep for 1 second

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        listener_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main(topic_name='/scan', queue_size=10)
