import pyaudio
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import args
import utils
import global_constants as gc


class VrAudioPublisher(Node):
    def __init__(self, **kwargs):
        # topic_name: str,
        # sample_rate: int,
        # channels: int,
        # chunk_size: int,
        # format: str,
        # verbose: int = 0,
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'vr_audio_publisher.yaml',
            read_from_command_line=False,
            **kwargs,
        )

        super().__init__('vr_audio_publisher')
        self.publisher = self.create_publisher(UInt8MultiArray, parameters['topic_name'], 10)
        self._running = True

        self.pa = pyaudio.PyAudio()
        device_index = self._find_respeaker_device()

        self.stream = self.pa.open(
            format=parameters['format'],
            channels=parameters['channels'],
            rate=parameters['sample_rate'],
            input=True,
            input_device_index=device_index,
            frames_per_buffer=parameters['chunk_size'],
            stream_callback=self._audio_callback,
        )
        self.stream.start_stream()
        print(f'VrAudioPublisher started on device index {device_index}')

    def _find_respeaker_device(self) -> int:
        """Find the ReSpeaker device index automatically."""
        for i in range(self.pa.get_device_count()):
            info = self.pa.get_device_info_by_index(i)
            if 'ReSpeaker' in info['name'] and info['maxInputChannels'] > 0:
                print(f'Found ReSpeaker at index {i}: {info["name"]}')
                return i
        print('ReSpeaker not found, using default input device')
        return None  # falls back to system default

    def _audio_callback(self, in_data, frame_count, time_info, status):
        if self._running:
            msg = UInt8MultiArray()
            msg.data = list(in_data)
            self.publisher.publish(msg)
        return (None, pyaudio.paContinue)

    def destroy(self):
        self._running = False
        self.stream.stop_stream()
        self.stream.close()
        self.pa.terminate()
        self.destroy_node()


class ThreadedVrAudioPublisher:
    def __init__(self, **kwargs):
        # topic_name: str,
        # sample_rate: int,
        # channels: int,
        # chunk_size: int,
        # format: str,
        # verbose: int = 0,
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'vr_audio_publisher.yaml',
            read_from_command_line=False,
            **kwargs,
        )

        self.verbose = parameters['verbose']
        self._node = None
        self._thread = None
        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = VrAudioPublisher(
                topic_name=parameters['topic_name'],
                sample_rate=parameters['sample_rate'],
                channels=parameters['channels'],
                chunk_size=parameters['chunk_size'],
                format=parameters['format'],
            )
            self._thread = threading.Thread(
                target=rclpy.spin,
                name='vr_audio_publisher_thread',
                args=(self._node,),
                daemon=True,
            )
            self._thread.start()
        except Exception as e:
            utils.print_exception(exception=e, message='VrAudioPublisher init error')

    def __del__(self):
        if self._node:
            self._node.destroy()
