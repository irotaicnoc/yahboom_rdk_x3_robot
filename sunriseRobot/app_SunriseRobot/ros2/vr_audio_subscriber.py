import pyaudio
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import args
import utils
import global_constants as gc


class VrAudioSubscriber(Node):
    def __init__(self, **kwargs):
        # topic_name: str,
        # queue_size: int,
        # sample_rate: int,
        # chunk_size: int,
        # channels: int,
        # format: str,
        # input_expiration_time: float = 0.5,
        # verbose: int = 0,
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'vr_audio_subscriber.yaml',
            read_from_command_line=False,
            **kwargs,
        )

        super().__init__('vr_audio_subscriber')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            parameters['topic_name'],
            self._audio_callback,
            parameters['queue_size'],
        )
        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(
            format=parameters['format'],
            channels=parameters['channels'],
            rate=parameters['sample_rate'],
            output=True,
            frames_per_buffer=parameters['chunk_size'],
        )
        print(f'VrAudioSubscriber started, listening on {parameters["topic_name"]}')

    def _audio_callback(self, msg: UInt8MultiArray):
        print('Received audio from vr')
        self.stream.write(bytes(msg.data))

    def destroy(self):
        self.stream.stop_stream()
        self.stream.close()
        self.pa.terminate()
        self.destroy_node()


class ThreadedVrAudioSubscriber:
    def __init__(self, **kwargs):
        # topic_name: str,
        # queue_size: int,
        # sample_rate: int,
        # channels: int,
        # chunk_size: int,
        # format: str,
        # input_expiration_time: float = 0.5,
        # verbose: int = 0,
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'vr_audio_subscriber.yaml',
            read_from_command_line=False,
            **kwargs,
        )

        self.verbose = parameters['verbose']
        self._node = None
        self._thread = None
        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = VrAudioSubscriber(
                topic_name=parameters['topic_name'],
                queue_size=parameters['queue_size'],
                sample_rate=parameters['sample_rate'],
                channels=parameters['channels'],
                chunk_size=parameters['chunk_size'],
                format=parameters['format'],
                input_expiration_time=parameters['input_expiration_time'],
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
