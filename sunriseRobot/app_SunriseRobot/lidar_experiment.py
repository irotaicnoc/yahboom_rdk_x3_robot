import time

from physical_accessories import lidar_listener


def obstacle_sensor():
    """
    This function initializes the LidarListener class, which subscribes to a LIDAR topic and processes the incoming data.
    It checks for obstacles in the robot's path and publishes velocity commands accordingly.
    """
    # Initialize the LidarListener
    lidar_listener_node = lidar_listener.ThreadedLidarListener(
        topic_name='/scan',
        queue_size=10,
        laser_angle=40,
        response_dist=1,
        verbose=3,
    )
    while True:
        time.sleep(0.5)
        obstacle_right, obstacle_left, obstacle_front = lidar_listener_node.get_obstacle_data()
        if obstacle_right:
            print('Obstacle detected on the right side')
        if obstacle_left:
            print('Obstacle detected on the left side')
        if obstacle_front:
            print('Obstacle detected in front')

    lidar_listener_node.delete_listener()


if __name__ == '__main__':
    '''
    Main function to run the obstacle sensor.
    '''
    try:
        obstacle_sensor()
    except KeyboardInterrupt:
        print('Obstacle sensor stopped by user.')
    except Exception as e:
        print('Vision agent error:')
        print(e)
        print(e.__traceback__)