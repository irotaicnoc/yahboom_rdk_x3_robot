import time
import array
import threading

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import UInt8MultiArray

import args
import global_constants as gc

# bytes per sample, used to chunk a clip into whole samples for pacing
_FORMAT_WIDTH = {
    'S16_LE':   2,
    'S32_LE':   4,
    'S8':       1,
    'U8':       1,
    'FLOAT_LE': 4,
}


class TtsToAppPublisher(Node):
    """
    Publishes app-originated TTS responses to the connected app on a dedicated topic (/audio_to_app), so the
    robot itself stays silent for those responses. It is the counterpart of AudioBridgeServer's speaker
    routing: when a voice request came from the app (robot_head.last_voice_source == 'app'), the bridge hands
    the whole TTS clip to robot_head.tts_to_app_clips instead of playing it on the ReSpeaker, and this node
    forwards it to the app.

    It is the reverse of VrAudioPublisher: rather than relaying the robot's microphone to the app, it plays the
    Jetson's synthesized speech straight to the app as discrete clips. It never touches the ReSpeaker (no
    hardware capture or playback); it only forwards PCM already produced on the Jetson.

    Wire format (must match the app-side subscriber):
      - topic: /audio_to_app, message: std_msgs/UInt8MultiArray
      - payload: raw 24 kHz mono int16 little-endian PCM, one chunk per message (same rate/format as the
        Jetson TTS output; see audio_bridge_server.yaml speaker_* params).
    Unlike the live mic streams (BEST_EFFORT, drop stale chunks), this uses RELIABLE QoS: a TTS clip is a
    discrete message where every chunk matters, so none should be dropped.
    """

    def __init__(self, robot_head=None, **kwargs):
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'tts_to_app_publisher.yaml',
            read_from_command_line=False,
            **kwargs,
        )
        super().__init__('tts_to_app_publisher')
        # robot_head.tts_to_app_clips is the handoff queue filled by the audio bridge.
        self.robot_head = robot_head
        self.verbose = parameters['verbose']
        self.sample_rate = parameters['sample_rate']
        self.width = _FORMAT_WIDTH[parameters['format']]  # bytes per sample
        self.chunk_size = parameters['chunk_size']        # samples per published message
        # bytes per published chunk and the wall-clock duration it represents, used to pace publishing at real
        # time so a long clip cannot outrun the subscriber.
        self._chunk_bytes = self.chunk_size * self.width
        self._chunk_period = self.chunk_size / self.sample_rate

        # RELIABLE + KEEP_LAST: deliver every chunk of a discrete clip, in order.
        audio_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=parameters['queue_size'],
        )
        self.publisher = self.create_publisher(UInt8MultiArray, parameters['topic_name'], audio_qos)

        self._running = True
        self._thread = threading.Thread(target=self._drain_loop, name='tts_to_app_publisher_drain', daemon=True)
        self._thread.start()
        if self.verbose >= 1:
            print(f'TtsToAppPublisher started on {parameters["topic_name"]}')

    def _drain_loop(self):
        """Pop whole TTS clips handed over by the audio bridge and publish them chunk by chunk at real time."""
        while self._running:
            clip = None
            if self.robot_head is not None:
                try:
                    clip = self.robot_head.tts_to_app_clips.popleft()
                except IndexError:
                    clip = None
            if clip:
                self._publish_clip(clip)
            else:
                time.sleep(0.02)

    def _publish_clip(self, pcm_bytes: bytes):
        for offset in range(0, len(pcm_bytes), self._chunk_bytes):
            if not self._running:
                return
            chunk = pcm_bytes[offset:offset + self._chunk_bytes]
            msg = UInt8MultiArray()
            # array.array('B', ...) copies the raw bytes in C; list(chunk) would build a Python int per byte,
            # which is needlessly slow on the RDK X3 (same note as in VrAudioPublisher).
            msg.data = array.array('B', chunk)
            self.publisher.publish(msg)
            # pace at real time so we do not outrun the subscriber (a long response is many chunks).
            time.sleep(self._chunk_period)

    def destroy(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        # drop any TTS that was not sent, so a stale answer cannot surface in a future session.
        if self.robot_head is not None:
            self.robot_head.tts_to_app_clips.clear()
        self.destroy_node()
