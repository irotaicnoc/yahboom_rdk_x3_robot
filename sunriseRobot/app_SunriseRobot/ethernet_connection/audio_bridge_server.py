import time
import socket
import threading

import alsaaudio
import numpy as np

import args
import utils
import global_constants as gc
from robot_link import protocol

_FORMAT_TO_ALSA = {
    'S16_LE':   alsaaudio.PCM_FORMAT_S16_LE,
    'S32_LE':   alsaaudio.PCM_FORMAT_S32_LE,
    'S8':       alsaaudio.PCM_FORMAT_S8,
    'U8':       alsaaudio.PCM_FORMAT_U8,
    'FLOAT_LE': alsaaudio.PCM_FORMAT_FLOAT_LE,
}

# bytes per sample, used to chunk playback into whole ALSA periods
_FORMAT_WIDTH = {
    'S16_LE':   2,
    'S32_LE':   4,
    'S8':       1,
    'U8':       1,
    'FLOAT_LE': 4,
}


class AudioBridgeServer:
    """
    Intra-robot audio bridge between the RDK X3 (which physically owns the single ReSpeaker Mic Array v2.0,
    microphone + speaker output) and the Jetson Nano (which runs the Google voice interaction).

    Two independent, unidirectional TCP streams on the wired link, each on its own port (mirroring the
    per-concern style of the JSON command channel and the arm camera relay). The RDK X3 is the server for
    both; the Jetson connects out, so it only ever needs the RDK X3 address.

      - Microphone stream (mic_stream_port), RDK X3 -> Jetson:
        the audio is push-to-talk gated. While a voice session is held on the robot_head (South button), the
        RDK X3 streams the selected source to the Jetson, one frame per audio chunk:
            [1 byte is_voice flag][4-byte big-endian PCM length][mono int16 PCM bytes]
          * 'robot' source (joystick South): the ReSpeaker ALSA capture (6 interleaved int16 channels; channel
            0 is the processed AEC + beamforming + noise-suppressed channel), opened on demand and released
            when idle so the device stays free for the VR audio nodes.
          * 'app' source (app button A): the phone/headset mic the app streams to /audio_from_vr, handed over
            by VrAudioSubscriber through robot_head.app_mic_frames; the ReSpeaker is not touched.
        Frames captured while held carry is_voice=True; between sessions is_voice=False frames are sent, and
        the first one after a hold tells the Jetson the button was released, so it finalizes the clip.

      - Speaker playback (speaker_playback_port), Jetson -> RDK X3:
        the Jetson sends TTS audio as framed PCM ([4-byte big-endian length][PCM bytes]) and the RDK X3 plays
        it through the ReSpeaker output, so the ReSpeaker echo canceller has it as a loopback reference and the
        microphone does not re-record the robot's own voice.

    KNOWN LIMITATION: the ALSA hw: capture of the ReSpeaker is exclusive. The VR audio publisher
    (ros2/vr_audio_publisher.py) opens the same device while a VR headset is connected, so the microphone
    stream cannot capture at the same time. If both are active, whichever opens the capture second fails and
    retries; this server logs a warning and drops the mic connection so the Jetson can retry later. Supporting
    simultaneous VR + voice interaction would require a single capturer that fans the processed channel out to
    both consumers. TODO: refactor to a shared capturer if simultaneous use becomes a requirement.
    """

    def __init__(self, robot_head=None, **kwargs):
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'audio_bridge_server.yaml',
            read_from_command_line=False,
            **kwargs,
        )
        # robot_head carries the push-to-talk session state (which source, if any, is active).
        self.robot_head = robot_head
        self.enabled = parameters['enabled']
        self.verbose = parameters['verbose']
        self.host = parameters['host']
        self.mic_stream_port = parameters['mic_stream_port']
        self.speaker_playback_port = parameters['speaker_playback_port']

        self.mic_sample_rate = parameters['mic_sample_rate']
        self.mic_capture_channels = parameters['mic_capture_channels']
        self.mic_processed_channel = parameters['mic_processed_channel']
        self.mic_format = parameters['mic_format']
        self.mic_chunk_size = parameters['mic_chunk_size']

        self.speaker_sample_rate = parameters['speaker_sample_rate']
        self.speaker_channels = parameters['speaker_channels']
        self.speaker_format = parameters['speaker_format']
        self.speaker_chunk_size = parameters['speaker_chunk_size']

        # Cadence (seconds) of the is_voice=False keepalive frames sent while no push-to-talk session is held.
        self.idle_frame_interval = parameters['idle_frame_interval']

        self.retry_interval = parameters['retry_interval']
        self.is_active = False

    def start(self) -> None:
        if not self.enabled:
            if self.verbose >= 1:
                print('Audio bridge server disabled (enabled=False).')
            return
        self.is_active = True
        mic_thread = threading.Thread(target=self._mic_stream_accept_loop, name='audio_bridge_mic_stream')
        speaker_thread = threading.Thread(target=self._speaker_playback_accept_loop, name='audio_bridge_speaker')
        mic_thread.start()
        speaker_thread.start()
        if self.verbose >= 1:
            print(f'Audio bridge server started (mic stream on {self.host}:{self.mic_stream_port}, '
                  f'speaker playback on {self.host}:{self.speaker_playback_port}).')

    def stop(self) -> None:
        self.is_active = False

    # ------------------------------------------------------------------ shared helpers

    def _open_server_socket(self, port: int):
        """Bind and listen on the wired link, retrying until it succeeds or the server is stopped."""
        while self.is_active:
            try:
                server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # prevent "Address already in use" on quick restarts
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server_socket.bind((self.host, port))
                server_socket.listen(1)
                if self.verbose >= 2:
                    print(f'Audio bridge listening on {self.host}:{port}')
                return server_socket
            except OSError as e:
                utils.print_exception(exception=e, message=f'Audio bridge could not bind {self.host}:{port}')
                if self.verbose >= 1:
                    print(f'\tRetrying in {self.retry_interval}s...')
                time.sleep(self.retry_interval)
        return None

    @staticmethod
    def _find_respeaker_card_index() -> int:
        """Find the ReSpeaker ALSA card index (same logic as the VR audio publisher)."""
        for idx in alsaaudio.card_indexes():
            short_name, long_name = alsaaudio.card_name(idx)
            if 'ReSpeaker' in long_name or 'ReSpeaker' in short_name or 'ArrayUAC' in short_name:
                return idx
        raise RuntimeError('ReSpeaker device not found. Check USB connection.')

    def _ensure_capture(self, capture):
        """Open the ReSpeaker capture if it is not already open. Returns the PCM, or None if it could not be
        opened (most likely the VR audio publisher already holds the exclusive hw: capture)."""
        if capture is not None:
            return capture
        try:
            card_idx = self._find_respeaker_card_index()
            return alsaaudio.PCM(
                type=alsaaudio.PCM_CAPTURE,
                mode=alsaaudio.PCM_NORMAL,
                device=f'hw:{card_idx},0',
                rate=self.mic_sample_rate,
                channels=self.mic_capture_channels,
                format=_FORMAT_TO_ALSA[self.mic_format],
                periodsize=self.mic_chunk_size,
            )
        except Exception as e:
            utils.print_exception(exception=e, message='Mic stream could not open ReSpeaker capture '
                                                        '(is a VR session using it?)')
            return None

    @staticmethod
    def _close_capture(capture):
        """Close the ReSpeaker capture if open and return None, freeing the device for the VR audio nodes."""
        if capture is not None:
            try:
                capture.close()
            except Exception:
                pass
        return None

    # ------------------------------------------------------------------ microphone stream (RDK X3 -> Jetson)

    def _mic_stream_accept_loop(self) -> None:
        server_socket = self._open_server_socket(self.mic_stream_port)
        if server_socket is None:
            return
        while self.is_active:
            try:
                connection, address = server_socket.accept()
            except OSError:
                break
            if self.verbose >= 1:
                print(f'Mic stream: Jetson connected from {address}')
            try:
                self._serve_mic_stream(connection)
            except Exception as e:
                utils.print_exception(exception=e, message='Mic stream connection error')
            finally:
                try:
                    connection.close()
                except OSError:
                    pass
                if self.verbose >= 1:
                    print('Mic stream: Jetson disconnected. Waiting for reconnection...')
        try:
            server_socket.close()
        except OSError:
            pass

    def _serve_mic_stream(self, connection) -> None:
        """
        Push-to-talk producer loop. The audio source is selected per session on the robot_head:
          - 'robot': the ReSpeaker capture (joystick South press), opened on demand and released when idle.
          - 'app':   the phone/headset mic frames arriving on /audio_from_vr (app button A), handed over by
                     VrAudioSubscriber through robot_head.app_mic_frames.
        Audio captured while a session is held is sent with is_voice=True (an empty is_voice=True heartbeat
        covers momentary gaps in the app source so a hold is never split). When no session is active
        is_voice=False frames are sent; the first one after a hold tells the Jetson the button was released,
        so it finalizes the clip.
        """
        capture = None
        try:
            while self.is_active:
                source = None
                if self.robot_head is not None and self.robot_head.voice_session_active:
                    source = self.robot_head.voice_session_source

                if source == 'robot':
                    capture = self._ensure_capture(capture)
                    if capture is None:
                        # device busy (a VR session holds it); behave as idle until it frees up
                        self._send_keepalive(connection)
                        continue
                    length, data = capture.read()
                    if length <= 0:
                        continue
                    # interleaved int16; keep only the processed channel (channel 0).
                    audio = np.frombuffer(data, dtype=np.int16)
                    mono = audio[self.mic_processed_channel::self.mic_capture_channels].tobytes()
                    protocol.send_audio_frame(connection, mono, is_voice=True)

                elif source == 'app':
                    # never hold the ReSpeaker during an app session (the app mic is used instead)
                    capture = self._close_capture(capture)
                    try:
                        frame = self.robot_head.app_mic_frames.popleft()
                    except IndexError:
                        frame = None
                    if frame:
                        protocol.send_audio_frame(connection, frame, is_voice=True)
                    else:
                        # session held but no app audio has arrived yet: send an empty 'held' heartbeat
                        # (is_voice=True) so the Jetson keeps the clip open until the button is released.
                        protocol.send_audio_frame(connection, b'', is_voice=True)
                        time.sleep(self.idle_frame_interval)

                else:  # no active session
                    capture = self._close_capture(capture)
                    self._send_keepalive(connection)
        finally:
            self._close_capture(capture)

    def _send_keepalive(self, connection) -> None:
        """Send one is_voice=False frame (button not held) and pace the idle loop. The first such frame after
        a hold is the Jetson's signal that the button was released."""
        protocol.send_audio_frame(connection, b'', is_voice=False)
        time.sleep(self.idle_frame_interval)

    # ------------------------------------------------------------------ speaker playback (Jetson -> RDK X3)

    def _speaker_playback_accept_loop(self) -> None:
        server_socket = self._open_server_socket(self.speaker_playback_port)
        if server_socket is None:
            return
        while self.is_active:
            try:
                connection, address = server_socket.accept()
            except OSError:
                break
            if self.verbose >= 1:
                print(f'Speaker playback: Jetson connected from {address}')
            try:
                self._serve_speaker_playback(connection)
            except Exception as e:
                utils.print_exception(exception=e, message='Speaker playback connection error')
            finally:
                try:
                    connection.close()
                except OSError:
                    pass
                if self.verbose >= 1:
                    print('Speaker playback: Jetson disconnected. Waiting for reconnection...')
        try:
            server_socket.close()
        except OSError:
            pass

    def _open_playback(self):
        # 'default' (not hw:) so playback is mixed through the ReSpeaker output alongside the VR audio.
        return alsaaudio.PCM(
            type=alsaaudio.PCM_PLAYBACK,
            mode=alsaaudio.PCM_NORMAL,
            device='default',
            rate=self.speaker_sample_rate,
            channels=self.speaker_channels,
            format=_FORMAT_TO_ALSA[self.speaker_format],
            periodsize=self.speaker_chunk_size,
        )

    def _serve_speaker_playback(self, connection) -> None:
        playback = None
        # one ALSA period in bytes; the Jetson sends a whole utterance at once, so we write it out period by
        # period (padding the last partial period with silence) which is robust across pyalsaaudio versions.
        period_bytes = self.speaker_chunk_size * self.speaker_channels * _FORMAT_WIDTH[self.speaker_format]
        try:
            playback = self._open_playback()
            while self.is_active:
                pcm_bytes = protocol.recv_message(connection)
                if pcm_bytes is None:
                    return  # peer closed the connection
                if pcm_bytes == b'':
                    continue  # zero-length frame, nothing to play
                for offset in range(0, len(pcm_bytes), period_bytes):
                    period = pcm_bytes[offset:offset + period_bytes]
                    if len(period) < period_bytes:
                        period = period + b'\x00' * (period_bytes - len(period))
                    try:
                        playback.write(period)
                    except alsaaudio.ALSAAudioError:
                        # recover from underrun and drop this period
                        try:
                            playback.close()
                        except Exception:
                            pass
                        playback = self._open_playback()
        finally:
            if playback is not None:
                try:
                    playback.close()
                except Exception:
                    pass
