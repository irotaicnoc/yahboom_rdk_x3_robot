import os
import copy
import time
import numpy as np

from physical_accessories import lidar_listener


def obstacle_sensor():
    print('Experiment started')
    print('Initializing LidarListener...')
    # Initialize the LidarListener
    lidar_listener_node = lidar_listener.ThreadedLidarListener(
        topic_name='scan',
        queue_size=10,
        response_dist=0.3,
        verbose=3,
    )
    print('LidarListener initialized')
    ascii_circle = [[' ' for _ in range(20)] for _ in range(20)]
    for i in range(20):
        for j in range(20):
            if (i - 10) ** 2 + (j - 10) ** 2 <= 100:
                ascii_circle[i][j] = '.'

    try:
        counter = 0
        while True:
            os.system('clear')
            print(f'Counter: {counter}')
            canvas = copy.deepcopy(ascii_circle)
            lidar_data = lidar_listener_node.read_lidar_data()
            if lidar_data is not None:
                _, obstacles_by_sector, average_distance_by_sector = lidar_data
                # print an ascii art circle and add detected obstacles as 'X' using their direction and distance

                for sector_num in range(len(obstacles_by_sector)):
                    if obstacles_by_sector[sector_num]:
                        angle_grad = sector_num * lidar_listener_node.sector_angle
                        angle_grad = (angle_grad + 90) % 360
                        angle_rad = np.deg2rad(angle_grad)
                        distance = average_distance_by_sector[sector_num]
                        x = int(10 + distance * 40 * np.cos(angle_rad))
                        y = int(10 - distance * 40 * np.sin(angle_rad))
                        if 0 <= x < 20 and 0 <= y < 20:
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
