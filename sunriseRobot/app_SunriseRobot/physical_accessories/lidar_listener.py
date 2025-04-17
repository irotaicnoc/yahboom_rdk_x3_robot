import os
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarListener(Node):
    def __init__(self, topic_name: str, queue_size: int, response_dist: float, sector_angle: float):
        super().__init__('lidar_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            topic_name,
            self.lidar_scan_callback,
            queue_size
        )
        self.lidar_data = None
        self.lidar_data_is_new = True
        self.response_dist = response_dist
        self.sector_angle = sector_angle
        self.number_of_sectors = int(360 / sector_angle)
        self.obstacles_by_sector = np.zeros(shape=self.number_of_sectors, dtype=bool)
        self.counter_by_sector = np.zeros(shape=self.number_of_sectors, dtype=int)
        self.average_distance_by_sector = np.zeros(shape=self.number_of_sectors, dtype=float)
        print(f'number_of_sectors: {self.number_of_sectors}')

        # narrower sectors (smaller angles) means less hits are required to detect an obstacle
        # self.obstacle_found_threshold = 5
        self.obstacle_found_threshold = int(sector_angle / 4)

    def lidar_scan_callback(self, msg: LaserScan) -> None:
        # self.get_logger().info('Published processed lidar data')
        self.lidar_data = msg
        self.search_obstacles(msg)
        self.lidar_data_is_new = True

    def search_obstacles(self, scan_data: LaserScan) -> None:
        if not isinstance(scan_data, LaserScan):
            return

        self.counter_by_sector[:] = 0
        temp_average_distance_by_sector = np.zeros(shape=self.number_of_sectors, dtype=float)
        ranges = np.array(scan_data.ranges)
        for i in range(len(ranges)):
            if ranges[i] < self.response_dist:
                angle = (scan_data.angle_min + scan_data.angle_increment * i) * 180 / np.pi
                assert 0 <= angle <= 360, f'Angle {angle} is out of range [0, 360]'
                # if angle > 180:
                #     angle = angle - 360
                sector_num = int(angle / self.sector_angle)
                self.counter_by_sector[sector_num] += 1
                temp_average_distance_by_sector[sector_num] += ranges[i]

        for sector_num in range(len(self.obstacles_by_sector)):
            if self.counter_by_sector[sector_num] > self.obstacle_found_threshold:
                self.obstacles_by_sector[sector_num] = True
                self.average_distance_by_sector[sector_num] = (temp_average_distance_by_sector[sector_num] /
                                                               self.counter_by_sector[sector_num])
            else:
                self.obstacles_by_sector[sector_num] = False
                self.average_distance_by_sector[sector_num] = -1

    def read_lidar_data(self) -> tuple:
        if self.lidar_data_is_new:
            self.lidar_data_is_new = False
            return self.lidar_data, self.obstacles_by_sector, self.average_distance_by_sector
        else:
            return None


class ThreadedLidarListener:
    def __init__(self,
                 topic_name: str,
                 queue_size: int = 10,
                 response_dist: float = 0.3,
                 sector_angle: float = 20,
                 verbose: int = 0,
                 ):
        self.topic_name = topic_name
        self.queue_size = queue_size
        self.sector_angle = sector_angle
        self.response_dist = response_dist
        self.lidar_listener_node = None
        self.spin_thread = None
        self.verbose = verbose
        try:
            rclpy.init()
            self.lidar_listener_node = LidarListener(
                topic_name=topic_name,
                queue_size=queue_size,
                response_dist=response_dist,
                sector_angle=sector_angle,
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

            os.system('gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash;cd /root/marco_ros2_ws/;'
                      'source install/local_setup.bash;ros2 launch oradar_lidar ms200_scan.launch.py;exec bash"')

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

    def read_lidar_data(self) -> tuple:
        if self.lidar_listener_node is not None:
            return self.lidar_listener_node.read_lidar_data()

        if self.verbose >= 2:
            print('Lidar listener node is None')
        return None

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