import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import threading
import numpy as np


class LidarListener(Node):
    def __init__(self, topic_name: str, queue_size: int, laser_angle: float, response_dist: float):
        super().__init__('lidar_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            topic_name,
            self.lidar_scan_callback,
            queue_size
        )
        self.lidar_data = None
        self.lidar_data_is_new = True
        self.laser_angle = laser_angle
        self.response_dist = response_dist
        self.obstacle_right = False
        self.obstacle_left = False
        self.obstacle_front = False
        self.FRONT_CONE_ANGLE = 20
        self.OBSTACLE_FOUND_THRESHOLD = 10

    def lidar_scan_callback(self, msg: LaserScan) -> None:
        # self.get_logger().info('Published processed lidar data')
        self.lidar_data = msg
        self.search_obstacles(msg)
        self.lidar_data_is_new = True

    def search_obstacles(self, scan_data: LaserScan) -> None:
        if not isinstance(scan_data, LaserScan):
            return
        right_warnings = 0
        left_warnings = 0
        front_warnings = 0

        ranges = np.array(scan_data.ranges)
        for i in range(len(ranges)):
            if ranges[i] < self.response_dist:
                angle = (scan_data.angle_min + scan_data.angle_increment * i) * 180 / np.pi
                if angle > 180:
                    angle = angle - 360
                if -self.laser_angle < angle < -self.FRONT_CONE_ANGLE:
                    right_warnings += 1
                elif abs(angle) <= self.FRONT_CONE_ANGLE:
                    front_warnings += 1
                elif self.FRONT_CONE_ANGLE < angle < self.laser_angle:
                    left_warnings += 1

        if right_warnings > self.OBSTACLE_FOUND_THRESHOLD:
            self.obstacle_right = True
        else:
            self.obstacle_right = False
        if left_warnings > self.OBSTACLE_FOUND_THRESHOLD:
            self.obstacle_left = True
        else:
            self.obstacle_left = False
        if front_warnings > self.OBSTACLE_FOUND_THRESHOLD:
            self.obstacle_front = True
        else:
            self.obstacle_front = False

        # if self.Joy_active == True or self.Switch == False:
        #     if self.Moving == True:
        #         self.pub_vel.publish(Twist())
        #         self.Moving = not self.Moving
        #     return
        #
        # self.Moving = True
        # twist = Twist()
        # if self.obstacle_front and self.obstacle_left and self.obstacle_right:
        #     print('1, there are obstacles in the left and right, turn right')
        #     twist.linear.x = self.linear
        #     twist.angular.z = -self.angular
        #     self.pub_vel.publish(twist)
        #     time.sleep(0.2)
        #
        # elif self.obstacle_front and not self.obstacle_left and self.obstacle_right:
        #     print('2, there is an obstacle in the middle right, turn left')
        #     twist.linear.x = self.linear
        #     twist.angular.z = self.angular
        #     self.pub_vel.publish(twist)
        #     time.sleep(0.2)
        #     if self.obstacle_left and not self.obstacle_right:
        #         twist.linear.x = self.linear
        #         twist.angular.z = -self.angular
        #         self.pub_vel.publish(twist)
        #         time.sleep(0.5)
        #
        # elif self.obstacle_front and self.obstacle_left and not self.obstacle_right:
        #     print('4. there is an obstacle in the middle left, turn right')
        #     twist.linear.x = self.linear
        #     twist.angular.z = -self.angular
        #     self.pub_vel.publish(twist)
        #     time.sleep(0.2)
        #     if not self.obstacle_left and self.obstacle_right:
        #         twist.linear.x = self.linear
        #         twist.angular.z = self.angular
        #         self.pub_vel.publish(twist)
        #         time.sleep(0.5)
        #
        # elif self.obstacle_front and not self.obstacle_left and not self.obstacle_right:
        #     print('6, there is an obstacle in the middle, turn left')
        #     twist.linear.x = self.linear
        #     twist.angular.z = self.angular
        #     self.pub_vel.publish(twist)
        #     time.sleep(0.2)
        #
        # elif not self.obstacle_front and self.obstacle_left and self.obstacle_right:
        #     print('7. there are obstacles on the left and right, turn right')
        #     twist.linear.x = self.linear
        #     twist.angular.z = -self.angular
        #     self.pub_vel.publish(twist)
        #     time.sleep(0.4)
        #
        # elif not self.obstacle_front and self.obstacle_left and not self.obstacle_right:
        #     print('8, there is an obstacle on the left, turn right')
        #     twist.linear.x = self.linear
        #     twist.angular.z = -self.angular
        #     self.pub_vel.publish(twist)
        #     time.sleep(0.2)
        #
        # elif not self.obstacle_front and not self.obstacle_left and self.obstacle_right:
        #     print('9, there is an obstacle on the right, turn left')
        #     twist.linear.x = self.linear
        #     twist.angular.z = self.angular
        #     self.pub_vel.publish(twist)
        #     time.sleep(0.2)
        #
        # elif not self.obstacle_front and not self.obstacle_left and not self.obstacle_right:
        #     print('10, no obstacles, go forward')
        #     twist.linear.x = self.linear
        #     twist.angular.z = 0.0
        #     self.pub_vel.publish(twist)

    def read_lidar_data(self):
        if self.lidar_data_is_new:
            self.lidar_data_is_new = False
            return self.lidar_data
        else:
            return None


class ThreadedLidarListener:
    def __init__(self,
                 topic_name: str,
                 queue_size: int = 10,
                 laser_angle: float = 40.0,
                 response_dist: float = 0.8,
                 verbose: int = 0,
                 ):
        self.topic_name = topic_name
        self.queue_size = queue_size
        self.lidar_listener_node = None
        self.spin_thread = None
        self.verbose = verbose
        try:
            rclpy.init()
            self.lidar_listener_node = LidarListener(
                topic_name=topic_name,
                queue_size=queue_size,
                laser_angle=laser_angle,
                response_dist=response_dist
            )
            # Spin the node in a separate thread
            self.spin_thread = threading.Thread(
                target=rclpy.spin,
                name='lidar_listener',
                args=(self.lidar_listener_node,)
            )
            self.spin_thread.start()
            if self.verbose >= 1:
                print('Lidar listener created and listening')

        except Exception as e:
            print('Lidar listener creation error:')
            print(e)
            print(e.__traceback__)
            try:
                self.lidar_listener_node.destroy_node()
                rclpy.shutdown()
            except:
                try:
                    rclpy.shutdown()
                except:
                    pass

    def read_lidar_data(self) -> LaserScan:
        if self.lidar_listener_node is not None:
            return self.lidar_listener_node.read_lidar_data()

        if self.verbose >= 2:
            print('Lidar listener node is None')
        return None

    def get_obstacle_data(self) -> tuple:
        if self.lidar_listener_node is not None:
            return (
                self.lidar_listener_node.obstacle_right,
                self.lidar_listener_node.obstacle_left,
                self.lidar_listener_node.obstacle_front
            )
        if self.verbose >= 2:
            print('Lidar listener node is None')
        return False, False, False

    def delete_listener(self):
        if self.spin_thread is not None:
            if self.verbose >= 2:
                print('Check stopping lidar listener...')
            self.lidar_listener_node.destroy_node()
            rclpy.shutdown()
            self.spin_thread.join()
            if self.verbose >= 1:
                print('Lidar listener stopped')
        else:
            if self.verbose >= 1:
                print('Lidar listener not stopped, thread is already None')

    def __del__(self):
        self.delete_listener()