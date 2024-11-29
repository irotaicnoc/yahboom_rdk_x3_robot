import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
import math


class LaserScanToPointPublish(Node):
    def __init__(self):
        super().__init__('robot_pose_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10)
        self.sacn_point_publisher = self.create_publisher(
            Path,
            '/scan_points',
            10)

    def laser_scan_callback(self, msg):
        # print(msg)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        laser_scan = msg.ranges
        # print(laser_scan)
        laser_points = self.laser_scan_to_points(
            laser_scan=laser_scan,
            angle_min=angle_min,
            angle_increment=angle_increment,
        )
        self.sacn_point_publisher.publish(laser_points)

    @staticmethod
    def laser_scan_to_points(laser_scan, angle_min, angle_increment):
        points = []
        angle = angle_min
        laser_points = Path()

        for distance in laser_scan:
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            points.append(pose)
            angle += angle_increment
        laser_points.poses = points

        return laser_points


def main(args=None):
    rclpy.init(args=args)

    robot_laser_scan_publisher = LaserScanToPointPublish()

    rclpy.spin(robot_laser_scan_publisher)

    robot_laser_scan_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
