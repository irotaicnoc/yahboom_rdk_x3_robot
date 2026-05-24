import alsaaudio
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import UInt8MultiArray

import args
import utils
import global_constants as gc

_FORMAT_INFO = {
    'S16_LE':   (alsaaudio.PCM_FORMAT_S16_LE,   np.int16,   -2**15, 2**15 - 1),
    'S32_LE':   (alsaaudio.PCM_FORMAT_S32_LE,   np.int32,   -2**31, 2**31 - 1),
    'S8':       (alsaaudio.PCM_FORMAT_S8,       np.int8,    -2**7,  2**7 - 1),
    'U8':       (alsaaudio.PCM_FORMAT_U8,       np.uint8,    0,     2**8 - 1),   # center is 128
    'FLOAT_LE': (alsaaudio.PCM_FORMAT_FLOAT_LE, np.float32, -1.0,   1.0),
}


class VrAudioSubscriber(Node):
    def __init__(self, **kwargs):
        # topic_name: str,
        # queue_size: int,
        # sample_rate: int,
        # chunk_size: int,
        # channels: int,
        # format: str,
        # input_expiration_time: float = 0.5,
        # gain: float = 1.0,
        # verbose: int = 0,
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'vr_audio_subscriber.yaml',
            read_from_command_line=False,
            **kwargs,
        )
        super().__init__('vr_audio_subscriber')

        self.gain = parameters['gain']
        fmt_name = parameters['format']
        alsa_fmt, self.dtype, self.sample_min, self.sample_max = _FORMAT_INFO[fmt_name]
        self.is_unsigned = fmt_name == 'U8'

        # BEST_EFFORT + depth=1: for live audio, dropping a stale chunk is always better
        # than waiting for it. BEST_EFFORT is compatible with the endpoint's RELIABLE
        # publisher (a BEST_EFFORT sub accepts both RELIABLE and BEST_EFFORT pubs).
        audio_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=parameters['queue_size'],
        )
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            parameters['topic_name'],
            self._audio_callback,
            audio_qos,
        )
        self.pcm = alsaaudio.PCM(
            type=alsaaudio.PCM_PLAYBACK,
            mode=alsaaudio.PCM_NORMAL,
            device='default',
            rate=parameters['sample_rate'],
            channels=parameters['channels'],
            format=alsa_fmt,
            periodsize=parameters['chunk_size'],
        )
        print(f'VrAudioSubscriber started, listening on {parameters["topic_name"]}')

    def _audio_callback(self, msg: UInt8MultiArray):
        if self.gain == 1.0:
            data = bytes(msg.data)
        else:
            samples = np.frombuffer(bytes(msg.data), dtype=self.dtype)

            if self.is_unsigned:
                # uint8 PCM is centered at 128
                centered = samples.astype(np.float32) - 128.0
                amplified = np.clip(centered * self.gain, -128.0, 127.0) + 128.0
                out = amplified.astype(self.dtype)
            elif np.issubdtype(self.dtype, np.integer):
                # promote to int64 so the multiply can't overflow before clipping
                amplified = samples.astype(np.int64) * self.gain
                out = np.clip(amplified, self.sample_min, self.sample_max).astype(self.dtype)
            else:  # float32
                out = np.clip(samples * self.gain, -1.0, 1.0).astype(self.dtype)

            data = out.tobytes()

        try:
            self.pcm.write(data)
        except alsaaudio.ALSAAudioError:
            # Recover from underrun (EPIPE) and drop this chunk.
            try:
                self.pcm.prepare()
            except alsaaudio.ALSAAudioError:
                pass

    def destroy(self):
        try:
            self.pcm.close()
        except Exception:
            pass
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
        # gain: float = 1.0,
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
                gain=parameters['gain'],
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
