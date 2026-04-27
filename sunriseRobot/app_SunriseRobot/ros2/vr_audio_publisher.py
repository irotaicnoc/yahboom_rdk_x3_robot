import threading
import alsaaudio
import numpy as np

from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import args
import global_constants as gc

_FORMAT_TO_ALSA = {
    'S16_LE':   alsaaudio.PCM_FORMAT_S16_LE,
    'S32_LE':   alsaaudio.PCM_FORMAT_S32_LE,
    'S8':       alsaaudio.PCM_FORMAT_S8,
    'U8':       alsaaudio.PCM_FORMAT_U8,
    'FLOAT_LE': alsaaudio.PCM_FORMAT_FLOAT_LE,
}


# ReSpeaker Mic Array v2.0 (Seeed Studio)
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
        self._thread = None
        self.pcm = None

        card_short_name = self._find_respeaker_card()
        self.pcm = alsaaudio.PCM(
            type=alsaaudio.PCM_CAPTURE,
            mode=alsaaudio.PCM_NORMAL,
            device=f'hw:CARD={card_short_name},DEV=0',
            rate=parameters['sample_rate'],
            channels=parameters['channels'],
            format=_FORMAT_TO_ALSA[parameters['format']],
            periodsize=parameters['chunk_size'],
        )
        self._thread = threading.Thread(
            target=self._capture_loop,
            name='vr_audio_publisher_capture',
            daemon=True,
        )
        self._thread.start()
        print(f'VrAudioPublisher started on ALSA card "{card_short_name}"')

    @staticmethod
    def _find_respeaker_card() -> str:
        """Find the ReSpeaker ALSA card short name (e.g. 'ArrayUAC10')."""
        for idx in alsaaudio.card_indexes():
            short_name, long_name = alsaaudio.card_name(idx)
            if 'ReSpeaker' in long_name or 'ReSpeaker' in short_name or 'ArrayUAC' in short_name:
                return short_name
        raise RuntimeError('ReSpeaker device not found. Check USB connection.')

    def _capture_loop(self):
        try:
            while self._running:
                length, data = self.pcm.read()
                if not self._running or length <= 0:
                    continue
                # interleaved 6-channel int16, extract channel 5 (processed channel)
                audio = np.frombuffer(data, dtype=np.int16)
                mono = audio[5::6].tobytes()
                msg = UInt8MultiArray()
                msg.data = list(mono)
                self.publisher.publish(msg)
        except alsaaudio.ALSAAudioError as e:
            if self._running:
                print(f'VrAudioPublisher capture error: {e}')

    def destroy(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.pcm is not None:
            try:
                self.pcm.close()
            except Exception:
                pass
        self.destroy_node()
