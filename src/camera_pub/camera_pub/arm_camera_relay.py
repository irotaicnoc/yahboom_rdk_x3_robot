# ros2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# standard libraries
import time
import socket
import threading

# my libraries
from camera_pub import utils


def _recv_exactly(connection, num_bytes):
    """
    Receive exactly num_bytes from the socket.
    Returns the bytes, or None if the peer closed the connection before all bytes arrived.
    """
    buffer = b''
    while len(buffer) < num_bytes:
        chunk = connection.recv(num_bytes - len(buffer))
        if not chunk:
            return None
        buffer += chunk
    return buffer


class ArmCameraRelayNode(Node):
    """
    Republishes the arm camera (the USB camera physically attached to the Jetson Nano, mounted at the end
    of the robot arm) as a ROS2 topic, so the VR/mobile apps can subscribe to it exactly like the front
    camera (see camera_publisher.py / 'camera_stream').

    Transport (Option B): this node is the TCP *server* on the wired intra-robot link. The Jetson connects
    out to it, so the Jetson only ever needs to know the RDK X3 address (192.168.10.11), the same address
    it already uses for the command channel. The protocol is a simple request/response pull:
      - this node sends a 1-byte request, but only while there is at least one downstream subscriber;
      - the Jetson replies with a 4-byte big-endian length prefix followed by that many JPEG bytes
        (a length of 0 means "no frame available yet").
    Because requests are only sent while an app is subscribed, no frames cross the link unless an app is
    actually watching ("send when required by the app").
    """

    def __init__(self,
                 host: str,
                 port: int,
                 topic_name: str,
                 fps: int,
                 queue_size: int,
                 no_subscriber_poll_interval: float,
                 accept_retry_interval: float,
                 verbose: int = 0,
                 ):
        super().__init__('arm_camera_relay_node')
        self.host = host
        self.port = port
        self.topic_name = topic_name
        self.frame_interval = 1.0 / fps
        self.no_subscriber_poll_interval = no_subscriber_poll_interval
        self.accept_retry_interval = accept_retry_interval
        self.verbose = verbose

        self.publisher = self.create_publisher(CompressedImage, self.topic_name, queue_size)
        self.server_socket = None
        self.is_active = True
        self.message_counter = 0
        self.get_logger().info(f'Arm camera relay publishing on topic "{self.topic_name}"')

    def _open_server_socket(self) -> bool:
        while self.is_active and rclpy.ok():
            try:
                self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # prevent "Address already in use" error on quick restarts
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.server_socket.bind((self.host, self.port))
                self.server_socket.listen(1)
                self.get_logger().info(f'Arm camera relay listening on {self.host}:{self.port}')
                return True
            except OSError as error:
                self.get_logger().warn(f'Could not bind {self.host}:{self.port} ({error}). '
                                       f'Retrying in {self.accept_retry_interval}s...')
                time.sleep(self.accept_retry_interval)
        return False

    def run(self) -> None:
        if not self._open_server_socket():
            return
        while self.is_active and rclpy.ok():
            try:
                connection, address = self.server_socket.accept()
            except OSError:
                break
            self.get_logger().info(f'Jetson frame streamer connected from {address}')
            try:
                self._serve_client(connection)
            except Exception as error:
                self.get_logger().warn(f'Frame streamer connection error: {error}')
            finally:
                try:
                    connection.close()
                except OSError:
                    pass
                self.get_logger().info('Jetson frame streamer disconnected. Waiting for reconnection...')

    def _serve_client(self, connection) -> None:
        while self.is_active and rclpy.ok():
            # On-demand gate: only pull frames while an app (via ROS-TCP-Endpoint) is subscribed.
            if self.publisher.get_subscription_count() == 0:
                time.sleep(self.no_subscriber_poll_interval)
                continue

            connection.sendall(b'\x01')  # request one frame
            length_prefix = _recv_exactly(connection, 4)
            if length_prefix is None:
                return  # peer closed the connection
            frame_length = int.from_bytes(length_prefix, byteorder='big')
            if frame_length == 0:
                # no frame available on the Jetson yet
                time.sleep(self.frame_interval)
                continue
            jpeg_bytes = _recv_exactly(connection, frame_length)
            if jpeg_bytes is None:
                return  # peer closed mid-frame
            self._publish_jpeg(jpeg_bytes)
            time.sleep(self.frame_interval)

    def _publish_jpeg(self, jpeg_bytes: bytes) -> None:
        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.format = 'jpeg'
        message.data = jpeg_bytes
        self.publisher.publish(message)
        if self.verbose >= 2 and self.message_counter % 100 == 0:
            self.get_logger().info(f'Relayed arm camera frame {self.message_counter} ({len(jpeg_bytes)} bytes)')
        self.message_counter += 1

    def destroy_node(self):
        self.get_logger().info('Destroying arm camera relay node')
        self.is_active = False
        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except OSError:
                pass
            self.server_socket = None
        super().destroy_node()


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    kwargs = utils.args_from_yaml(
        config_path='/root/marco_ros2_ws/src/camera_pub/camera_pub/arm_camera_relay_config.yaml'
    )
    node = ArmCameraRelayNode(**kwargs)

    # The accept/pull loop runs in its own thread; rclpy.spin keeps the node (clock, publisher, graph) alive.
    relay_thread = threading.Thread(target=node.run, name='arm_camera_relay', daemon=True)
    relay_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
