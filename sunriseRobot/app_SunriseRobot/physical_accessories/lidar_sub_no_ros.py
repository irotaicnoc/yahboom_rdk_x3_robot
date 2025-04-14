import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray  # Or other suitable message type
import threading


class LidarListener(Node):
    def __init__(self):
        super().__init__('embedded_lidar_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with your lidar topic name
            self.scan_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Float32MultiArray,  # Choose an appropriate message type
            'processed_lidar_data',  # Name of the new topic
            10
        )
        self.processed_data = None  # Store the latest processed data

    def scan_callback(self, msg):
        processed_data = self.process_scan_data(msg)
        data_to_publish = Float32MultiArray()
        data_to_publish.data = processed_data
        self.publisher_.publish(data_to_publish)
        self.get_logger().info('Published processed lidar data')
        self.processed_data = processed_data  # Update the stored data

    def process_scan_data(self, scan_msg):
        # Example: Extracting range data
        ranges = list(scan_msg.ranges)
        return ranges

    def get_latest_processed_data(self):
        return self.processed_data


def spin_node(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    listener_node = LidarListener()

    # Spin the node in a separate thread so it doesn't block the main program
    spin_thread = threading.Thread(target=spin_node, args=(listener_node,))
    spin_thread.start()

    # Your main program logic can now run here
    try:
        while True:
            # Access the latest processed data from the node
            latest_data = listener_node.get_latest_processed_data()
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
    main()
