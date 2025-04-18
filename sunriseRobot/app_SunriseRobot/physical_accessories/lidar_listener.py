import os
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarListener(Node):
    def __init__(self,
                 topic_name: str,
                 queue_size: int,
                 response_dist: float,
                 sector_angle: float = None,
                 search_only_arc: list = None,
                 ):
        super().__init__('lidar_listener')
        assert sector_angle or search_only_arc is not None, \
            'Either "sector_angle" or "search_only_arc" must be provided'
        assert sector_angle or search_only_arc is None, \
            'Only one of "sector_angle" or "search_only_arc" can be provided'
        self.subscription = self.create_subscription(
            LaserScan,
            topic_name,
            self.lidar_scan_callback,
            queue_size
        )
        self.lidar_data = None
        self.raw_scan_is_new = True
        self.response_dist = response_dist
        # sector_angle attributes
        self.sector_angle = sector_angle
        self.number_of_sectors = int(360 / sector_angle)
        self.obstacles_by_sector = np.zeros(shape=self.number_of_sectors, dtype=bool)
        self.hit_counter_by_sector = np.zeros(shape=self.number_of_sectors, dtype=int)
        self.average_distance_by_sector = np.zeros(shape=self.number_of_sectors, dtype=float)
        self.obstacle_scan_is_new = True
        # search_only_arc attributes
        self.search_only_arc = search_only_arc
        self.obstacle_in_arc = False
        self.average_distance_in_arc = 0
        self.arc_scan_is_new = True

        # narrower sectors (smaller angles) means less hits are required to detect an obstacle
        # self.obstacle_found_threshold = 5
        if sector_angle is not None:
            self.obstacle_found_threshold = int(sector_angle / 4)
        elif search_only_arc is not None:
            self.obstacle_found_threshold = int(search_only_arc[1] - search_only_arc[0] / 4)

    def lidar_scan_callback(self, msg: LaserScan) -> None:
        # self.get_logger().info('Published processed lidar data')
        self.lidar_data = msg
        self.raw_scan_is_new = True
        if self.sector_angle is not None:
            self.search_obstacles(msg)
            self.obstacle_scan_is_new = True
        elif self.search_only_arc:
            self.scan_arc(msg)
            self.arc_scan_is_new = True

    def search_obstacles(self, scan_data: LaserScan) -> None:
        if not isinstance(scan_data, LaserScan):
            return
        self.hit_counter_by_sector[:] = 0
        temp_average_distance_by_sector = np.zeros(shape=self.number_of_sectors, dtype=float)
        ranges = np.array(scan_data.ranges)
        for i in range(len(ranges)):
            if ranges[i] < self.response_dist:
                angle = np.rad2deg(scan_data.angle_min + scan_data.angle_increment * i)
                assert 0 <= angle <= 360, f'Angle {angle} is out of range [0, 360]'
                sector_num = int(angle / self.sector_angle)
                self.hit_counter_by_sector[sector_num] += 1
                temp_average_distance_by_sector[sector_num] += ranges[i]

        for sector_num in range(len(self.obstacles_by_sector)):
            if self.hit_counter_by_sector[sector_num] > self.obstacle_found_threshold:
                self.obstacles_by_sector[sector_num] = True
                self.average_distance_by_sector[sector_num] = (temp_average_distance_by_sector[sector_num] /
                                                               self.hit_counter_by_sector[sector_num])
            else:
                self.obstacles_by_sector[sector_num] = False
                self.average_distance_by_sector[sector_num] = -1

    def scan_arc(self, scan_data: LaserScan) -> None:
        if not isinstance(scan_data, LaserScan):
            return
        hit_counter = 0
        temp_average_distance = 0
        ranges = np.array(scan_data.ranges)
        for i in range(len(ranges)):
            if ranges[i] < self.response_dist:
                angle = np.rad2deg(scan_data.angle_min + scan_data.angle_increment * i)
                if self.search_only_arc[0] <= angle <= self.search_only_arc[1]:
                    hit_counter += 1
                    temp_average_distance += ranges[i]

        if self.hit_counter > self.obstacle_found_threshold:
            self.obstacle_in_arc = True
            self.average_distance_in_arc = temp_average_distance / hit_counter
        else:
            self.obstacle_in_arc = False
            self.average_distance_in_arc = -1

    def get_raw_scan(self) -> LaserScan:
        if self.raw_scan_is_new:
            self.raw_scan_is_new = False
            return self.lidar_data
        return None

    def get_obstacles_by_sector(self) -> tuple:
        if self.obstacle_scan_is_new:
            self.obstacle_scan_is_new = False
            return self.obstacles_by_sector, self.average_distance_by_sector
        return None, None

    def get_obstacle_in_arc(self) -> tuple:
        if self.arc_scan_is_new:
            self.arc_scan_is_new = False
            return self.obstacle_in_arc, self.average_distance_in_arc
        return None, None


class ThreadedLidarListener:
    def __init__(self,
                 topic_name: str = 'scan',
                 queue_size: int = 5,
                 response_dist: float = 0.6,
                 sector_angle: float = 20,
                 search_only_arc: list = None,
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
                search_only_arc=search_only_arc,
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

    def get_raw_scan(self) -> LaserScan:
        if self.lidar_listener_node is not None:
            return self.lidar_listener_node.get_raw_scan()
        if self.verbose >= 2:
            print('Lidar listener node is None')
        return None

    def get_obstacles_by_sector(self) -> tuple:
        if self.lidar_listener_node is not None:
            return self.lidar_listener_node.get_obstacles_by_sector()
        if self.verbose >= 2:
            print('Lidar listener node is None')
        return None, None

    def get_obstacle_in_arc(self) -> tuple:
        if self.lidar_listener_node is not None:
            return self.lidar_listener_node.get_obstacle_in_arc()
        if self.verbose >= 2:
            print('Lidar listener node is None')
        return None, None

    def delete_listener(self):
        if self.spin_thread is not None:
            self.lidar_listener_node.destroy_node()
            rclpy.shutdown()
            self.spin_thread.join()
            if self.verbose >= 2:
                print('Lidar listener stopped')
        else:
            if self.verbose >= 2:
                print('Lidar listener not stopped, thread is already None')

    def __del__(self):
        self.delete_listener()
