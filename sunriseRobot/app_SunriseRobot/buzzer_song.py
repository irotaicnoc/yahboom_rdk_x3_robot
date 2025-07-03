import time

from robot_body import RobotBody

robot_body = RobotBody(com='/dev/ttyUSB0', baud_rate=115200, verbose=2)

# --- Define the "Notes" (Durations) for Super Mario Theme ---
# These are relative durations. You can adjust the 'tempo' by changing the base_duration.
# A quarter note might be 0.2 seconds, an eighth note 0.1, etc.
# These values are approximate and will need fine-tuning!

base_duration = 0.1  # This is your "tempo" - feel free to adjust!

# Durations for common note types relative to base_duration
WHOLE_NOTE = base_duration * 4
HALF_NOTE = base_duration * 2
QUARTER_NOTE = base_duration
EIGHTH_NOTE = base_duration * 0.5
SIXTEENTH_NOTE = base_duration * 0.25


# A quick way to define a "dot" (increases duration by 50%)
def dotted(note_duration):
    return note_duration * 1.5


# Function to play a single "note" (a buzz followed by an optional rest)
def play_buzz_note(buzz_duration, rest_duration=0):
    robot_body.set_beep(buzz_duration * 2)
    time.sleep(buzz_duration)
    if rest_duration > 0:
        time.sleep(rest_duration)


# --- Super Mario Bros. Main Theme (Simplified Rhythm) ---
# This is a very simplified rhythmic transcription.
# You'll need to listen to the song and map the "on" and "off" times.
# Each tuple represents (buzz_duration, rest_duration)
mario_theme_rhythm = [
    (EIGHTH_NOTE, EIGHTH_NOTE), # E
    (EIGHTH_NOTE, EIGHTH_NOTE), # E
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # E
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # C
    (EIGHTH_NOTE, EIGHTH_NOTE), # E
    (EIGHTH_NOTE, EIGHTH_NOTE), # G
    (0, EIGHTH_NOTE),           # Rest
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # G (octave lower, but same pitch for buzzer)
    (0, EIGHTH_NOTE),           # Rest
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # C
    (0, EIGHTH_NOTE),           # Rest
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # G
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # E
    (0, EIGHTH_NOTE),           # Rest
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # A
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # B
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # Bb (same pitch for buzzer)
    (EIGHTH_NOTE, EIGHTH_NOTE), # A
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # G
    (EIGHTH_NOTE, EIGHTH_NOTE), # E
    (EIGHTH_NOTE, EIGHTH_NOTE), # G
    (EIGHTH_NOTE, EIGHTH_NOTE), # A
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # F
    (EIGHTH_NOTE, EIGHTH_NOTE), # G
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # E
    (0, EIGHTH_NOTE),           # Rest
    (EIGHTH_NOTE, EIGHTH_NOTE), # C
    (EIGHTH_NOTE, EIGHTH_NOTE), # D
    (EIGHTH_NOTE, EIGHTH_NOTE), # B (octave lower, same pitch)
]


def play_rhythm(rhythm_sequence):
    for buzz_d, rest_d in rhythm_sequence:
        play_buzz_note(buzz_d, rest_d)

# --- Main Program ---
if __name__ == "__main__":
    try:
        print("Playing Super Mario rhythm...")
        play_rhythm(mario_theme_rhythm)
        print("Done!")

    except KeyboardInterrupt:
        print("\nExiting and cleaning up.")
