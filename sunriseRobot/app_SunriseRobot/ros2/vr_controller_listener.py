import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import utils


class VrControllerListener(Node):
    def __init__(self,
                 topic_name: str,
                 queue_size: int,
                 input_expiration_time: float = 0.5):
        super().__init__('vr_controller_listener')

        self.subscription = self.create_subscription(
            Joy,
            topic_name,
            self.input_callback,
            queue_size
        )
        self.controller_input_data = None
        self.msg_timestamp = 0
        self.input_expiration_time = input_expiration_time

    def input_callback(self, msg: Joy) -> None:
        self.controller_input_data = msg
        self.msg_timestamp = time.time()

    def get_latest_input(self) -> Joy:
        # Returns the latest message if it isn't too old
        if (time.time() - self.msg_timestamp) < self.input_expiration_time:
            return self.controller_input_data
        return None

    def get_axes_and_buttons(self) -> tuple:
        # Convenience method to directly get the arrays
        data = self.get_latest_input()
        if data:
            return data.axes, data.buttons
        return None, None


class ThreadedVrControllerListener:
    def __init__(self,
                 topic_name: str = '/vr_controller',
                 queue_size: int = 10,
                 input_expiration_time: float = 0.5,
                 verbose: int = 0):

        self.topic_name = topic_name
        self.queue_size = queue_size
        self.input_expiration_time = input_expiration_time
        self.controller_listener_node = None
        self.spin_thread = None
        self.verbose = verbose

        try:
            if not rclpy.ok():
                rclpy.init()
            self.controller_listener_node = VrControllerListener(
                topic_name=self.topic_name,
                queue_size=self.queue_size,
                input_expiration_time=self.input_expiration_time
            )

            # Spin the node in a separate thread
            self.spin_thread = threading.Thread(
                target=rclpy.spin,
                name='vr_controller_listener_thread',
                args=(self.controller_listener_node,)
            )
            self.spin_thread.start()

            # if self.verbose >= 1:
            print(f'VR Controller listener created and listening on {topic_name}')

        except Exception as e:
            utils.print_exception(exception=e, message='VR Controller listener creation error')
            try:
                if self.controller_listener_node:
                    self.controller_listener_node.destroy_node()
            except Exception:
                pass

    def get_latest_input(self) -> Joy:
        if self.controller_listener_node is not None:
            return self.controller_listener_node.get_latest_input()
        if self.verbose >= 2:
            print('VR Controller listener node is None')
        return None

    def get_axes_and_buttons(self) -> tuple:
        if self.controller_listener_node is not None:
            return self.controller_listener_node.get_axes_and_buttons()
        if self.verbose >= 2:
            print('VR Controller listener node is None')
        return None, None

    def delete_listener(self):
        if self.spin_thread is not None:
            self.controller_listener_node.destroy_node()
            self.spin_thread.join()
            if self.verbose >= 2:
                print('VR Controller listener stopped')
        else:
            if self.verbose >= 2:
                print('VR Controller listener not stopped, thread is already None')

    def __del__(self):
        self.delete_listener()