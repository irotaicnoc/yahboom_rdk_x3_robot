import pyaudio
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import utils


class VrAudioSubscriber(Node):
    def __init__(self,
                 topic_name: str,
                 queue_size: int,
                 sample_rate: int,
                 chunk_size: int,
                 channels: int,
                 format: str,
                 input_expiration_time: float = 0.5,
                 ):
        super().__init__('vr_audio_subscriber')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            topic_name,
            self._audio_callback,
            queue_size,
        )
        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(
            format=format,
            channels=channels,
            rate=sample_rate,
            output=True,
            frames_per_buffer=chunk_size,
        )
        print('VrAudioSubscriber started, listening on ' + topic_name)

    def _audio_callback(self, msg: UInt8MultiArray):
        self.stream.write(bytes(msg.data))

    def destroy(self):
        self.stream.stop_stream()
        self.stream.close()
        self.pa.terminate()
        self.destroy_node()


class ThreadedVrAudioSubscriber:
    def __init__(self,
                 topic_name: str,
                 queue_size: int,
                 sample_rate: int,
                 channels: int,
                 chunk_size: int,
                 format: str,
                 input_expiration_time: float = 0.5,
                 verbose: int = 0,
                 ):
        self.verbose = verbose
        self._node = None
        self._thread = None
        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = VrAudioSubscriber(
                topic_name=topic_name,
                queue_size=queue_size,
                sample_rate=sample_rate,
                channels=channels,
                chunk_size=chunk_size,
                format=format,
                input_expiration_time=input_expiration_time,
            )
            self._thread = threading.Thread(
                target=rclpy.spin,
                name='vr_audio_subscriber_thread',
                args=(self._node,),
                daemon=True,
            )
            self._thread.start()
        except Exception as e:
            utils.print_exception(exception=e, message='VrAudioSubscriber init error')

    def __del__(self):
        if self._node:
            self._node.destroy()
