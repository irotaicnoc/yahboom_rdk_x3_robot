import os
import copy
import time
import numpy as np

from physical_accessories import lidar_listener


def obstacle_sensor():
    print('Experiment started')
    print('Initializing LidarListener...')
    # Initialize the LidarListener
    response_dist = 1.0
    lidar_listener_node = lidar_listener.ThreadedLidarListener(
        topic_name='scan',
        queue_size=10,
        response_dist=response_dist,
        verbose=3,
    )
    print('LidarListener initialized')
    circle_radius = 10
    circle_diameter = circle_radius * 2
    dist_proportion = circle_radius / response_dist

    ascii_circle = [[' ' for _ in range(circle_diameter)] for _ in range(circle_diameter)]
    for i in range(circle_diameter):
        for j in range(circle_diameter):
            if (i - circle_radius) ** 2 + (j - circle_radius) ** 2 <= circle_radius**2:
                ascii_circle[i][j] = '.'

    try:
        counter = 0
        while True:
            os.system('clear')
            print(f'Counter: {counter}')
            canvas = copy.deepcopy(ascii_circle)
            # add the robot position
            canvas[circle_radius - 1][circle_radius] = '^'
            canvas[circle_radius][circle_radius] = '|'

            # add detected obstacles using their direction and distance
            lidar_data = lidar_listener_node.read_lidar_data()
            if lidar_data is not None:
                _, obstacles_by_sector, average_distance_by_sector = lidar_data
                for sector_num in range(len(obstacles_by_sector)):
                    if obstacles_by_sector[sector_num]:
                        angle_grad = sector_num * lidar_listener_node.sector_angle
                        angle_grad = (angle_grad + 90) % 360
                        angle_rad = np.deg2rad(angle_grad)
                        distance = average_distance_by_sector[sector_num]
                        x = int(circle_radius + distance * dist_proportion * np.cos(angle_rad))
                        y = int(circle_radius - distance * dist_proportion * np.sin(angle_rad))
                        if 0 <= x < circle_diameter and 0 <= y < circle_diameter:
                            canvas[y][x] = '#'

            for row in canvas:
                print(' '.join(row))
            time.sleep(1)
            counter += 1

    except KeyboardInterrupt:
        print('Experiment stopped by user.')
        lidar_listener_node.delete_listener()
    except Exception as e:
        print('Obstacle sensor error:')
        print(e)
        print(e.__traceback__)
        lidar_listener_node.delete_listener()


if __name__ == '__main__':
    try:
        obstacle_sensor()
    except KeyboardInterrupt:
        print('Obstacle sensor stopped by user.')
    except Exception as e:
        print('Vision agent error:')
        print(e)
        print(e.__traceback__)
