# Voice interaction as a push-to-talk button (Option 1)

Plan for turning the always-on voice interaction into a push-to-talk button, replacing the manual
horn on the South button. Robot-side plan (this repo); the app-side change is noted at the end and is
owned by Marco.

Status: robot side implemented on branch `claude` (2026-07-18). Still pending: the app-side change
(below) and flipping `enable_voice_interaction: True` on the Jetson once the Google API rework is done.

## Goal / behavior

- The South button becomes "talk to the AI", in every mode and sub-mode (overrides the old horn and
  the old South arm-position memory slot; E/W/N arm slots stay).
- Push-to-talk: hold to record, release to send. The recording window is defined by the hold, not by VAD.
- Source of the audio depends on where the press comes from:
  - Joystick South (`from_vr=False`): the robot's ReSpeaker mic.
  - App button A (`from_vr=True`, VR + mobile): the app's phone/headset mic, which the apps already
    stream to the `/audio_from_vr` ROS2 topic while a talk button is held.
- The app's existing dedicated push-to-talk button stays as-is: it is the speaker intercom
  (phone mic -> robot speakers). Only button A is wired to the AI.

## Design (Option 1: one mic topic, route by button)

Both the intercom PTT button and button A feed the same `/audio_from_vr` topic (there is a single mic
publisher in the app). The robot decides the destination by whether button A (South, from_vr) is
currently held:
- A held  -> route `/audio_from_vr` audio to the AI (do not play it on the speakers).
- A not held -> play `/audio_from_vr` on the speakers (intercom, current behavior).

The RDK X3 is the audio router. The Jetson has a single mic bridge (TCP, port 65434) and stays
source-agnostic: the RDK fills that bridge from the ReSpeaker (joystick session) or from the
`/audio_from_vr` chunks (app session), tagging frames `is_voice=True` while held. The Jetson's existing
`MicrophoneListener` already turns "voice then silence" into a reasoning request, so it needs no code
change. (The Jetson has no ROS2, which is why the RDK must forward the app audio rather than the Jetson
subscribing directly.)

Audio formats already line up (16 kHz mono S16_LE on both the ReSpeaker bridge and the app mic), so no
resampling is needed.

## Robot-side changes, file by file

1. `robot_head.py`
   - Add voice-session state near the buzzer block (around line 69):
     - `self.voice_session_active = False`
     - `self.voice_session_source = None`   # 'robot' | 'app'
     - `self.app_mic_frames = queue.Queue(maxsize=N)`  # app audio pending forward to the bridge
   - Add small helpers `start_voice_session(source)` / `stop_voice_session()` that set the above
     (guard against a second source starting while one is active: first-one-wins).
   - Keep `buzzer_is_active` / `buzzer_state_changed`: they are NOT only the manual horn, they are also
     used by `utils.finish_generic_process` (the "done" beep) and the physical-button test stubs. Only the
     horn call in `button_south` is removed.

2. `controllers/controller_interface.py`
   - Rewrite `button_south` (lines 110-122) to:
     - `def button_south(self, value: bool, from_vr: bool = False)`
     - `if value: robot_head.start_voice_session('app' if from_vr else 'robot')`
       `else: robot_head.stop_voice_session()`
     - Apply in all modes/sub-modes (no per-mode branching: voice everywhere).
   - Remove the `'button_south'` seed from `self.memorized_arm_position` (line 31). South no longer goes
     through `memorize_or_set_arm_position`, so it drops out of the arm-memory bookkeeping naturally.
     Sanity-check no code reads a South entry in `button_press_timestamp` / `one_time_check` afterward.
   - Optional: drive `led_3_pin` to a "listening" color while a session is active (matches the Jetson
     mic listener's LED intent).

3. `controllers/meta_quest_3_controller.py`
   - Line 68: pass the source: `self.controller_functions.button_south(bool(buttons[0]), from_vr=True)`.

4. `controllers/ps2_controller.py`
   - No change: line 109 calls `button_south(value)` and `from_vr` defaults to False.

5. `controllers/controller_loop.py`
   - No change. The buzzer block stays: it is the consumer that actuates `buzzer_is_active`, still used by
     `utils.finish_generic_process` and the physical-button test stubs.

6. `ethernet_connection/audio_bridge_server.py`  (the main change)
   - Constructor: accept `robot_head`.
   - Refactor `_serve_mic_stream` from "open ReSpeaker on connect + stream hardware VAD" to a
     session-driven, source-aware producer loop:
     - session active, source 'robot': ensure the ReSpeaker capture is open, read a chunk, send it
       with `is_voice=True`.
     - session active, source 'app': drain `robot_head.app_mic_frames`, forward chunks with
       `is_voice=True` (framing size need not match; the Jetson just appends bytes).
     - idle: close the ReSpeaker capture if open (frees it for the VR audio nodes), and send a short
       tail of `is_voice=False` frames so the Jetson closes any pending utterance.
   - Net effect: ReSpeaker is opened on-demand only for joystick sessions; app sessions never touch it.

7. `ros2/vr_audio_subscriber.py`
   - Constructor: accept `robot_head`.
   - In `_audio_callback`: if `robot_head.voice_session_active and robot_head.voice_session_source ==
     'app'`, push `data` into `robot_head.app_mic_frames` (route to the AI) and skip speaker playback;
     otherwise play on the speakers (intercom, current behavior).

8. `main_thread.py`
   - Pass `robot_head` to `AudioBridgeServer(...)` (around line 232).
   - Pass `robot_head` to `VrAudioSubscriber(...)` in `task_audio_from_vr` (around line 370).

## Config knobs (config file first, per project convention)

- `configs/audio_bridge_server.yaml`: optional knob(s) for idle/tail behavior and app-buffer size.
  Update the header comment: the ReSpeaker is now opened on-demand per session, not on connect.
- Jetson `configs/microphone_listener.yaml`: consider lowering `max_silence_duration` (e.g. 0.6-1.0) so
  the utterance closes quickly after release, and revisit `min_sentence_duration` for short commands.
- Jetson `configs/main_thread.yaml`: flip `enable_voice_interaction: True` only once the Google API
  rework is done. Keep False for now; the RDK-side plumbing above can be built and tested independently.

## App-side change (Marco owns)

- In the shared `com.marco.ros2-shared` package: make `IPushToTalkSource.IsTalking` true when the
  dedicated PTT button OR button A is pressed, so button A also opens the mic to `/audio_from_vr`.
  Keep the dedicated PTT button as intercom. Button A is already published on `/vr_controller`
  (`buttons[0]`), which is what the robot uses to route to the AI. Both apps inherit this via the
  shared package.

## Testing without the Google backend

- The Jetson does not connect the mic bridge while `enable_voice_interaction=False`, so test the RDK
  routing locally: add a temporary dump in the router that writes forwarded frames to a WAV.
  - Joystick South -> a clip from the robot ReSpeaker.
  - App button A -> a clip from the phone/headset mic.
  - Dedicated PTT button -> audio on the speakers only, no clip.
- Confirm the ReSpeaker is released when idle (the VR audio publisher can capture).

## Edge cases / notes

- Simultaneous joystick + app session: first-one-wins (or app-priority); guard in `start_voice_session`.
- Joystick session while a VR headset is connected: `VrAudioPublisher` may hold the ReSpeaker capture,
  so the robot-mic capture can fail. Existing known limitation; log and skip.
- Cross-topic skew (button-A-held on `/vr_controller` vs audio on `/audio_from_vr`): negligible for
  push-to-talk (press, then speak). If it ever bites, switch to Option 2 (app publishes AI audio to a
  separate `/audio_to_ai` topic; robot forwards that topic straight to the bridge, no button
  correlation).
- Latency after release is bounded by `max_silence_duration`; tune it for snappiness.

## Deferred / not doing now

- Option 2 (separate AI audio topic) kept as a fallback if Option 1 routing feels fiddly.
- Relocating a manual horn to a rocker-stick button (both `button_rocker_left/right` are currently
  no-ops) if the horn is ever wanted back.
