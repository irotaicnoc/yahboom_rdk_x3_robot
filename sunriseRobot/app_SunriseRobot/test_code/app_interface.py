import tkinter as tk
from tkinter import ttk
import random  # For simulating battery level and function calls


# Mock robot functions (replace with actual robot control code)
def move_forward():
    print("Robot: Moving forward")
    update_status("Moving forward...")


def move_backward():
    print("Robot: Moving backward")
    update_status("Moving backward...")


def turn_left():
    print("Robot: Turning left")
    update_status("Turning left...")


def turn_right():
    print("Robot: Turning right")
    update_status("Turning right...")


def stop_robot():
    print("Robot: Stopping")
    update_status("Stopping...")


def get_battery_level():
    # Simulate getting battery level from the robot
    return random.randint(0, 100)


def update_status(message):
    status_label.config(text=message)


def create_app():
    """Creates the main application window and its elements."""
    global root, battery_label, status_label  # Declare globals to be updated

    root = tk.Tk()
    root.title("Robot Controller")
    # Set the window size.  You may need to adjust these dimensions.
    root.geometry("480x320")  # Example: 480x320 resolution

    # Create a style for larger buttons
    style = ttk.Style()
    style.configure("LargeButton.TButton", font=("Arial", 16), padding=10)
    style.configure("SmallerButton.TButton", font=("Arial", 12), padding=8)  # for stop button

    # Battery Label
    battery_label = ttk.Label(root, text="Battery: N/A", font=("Arial", 14))
    battery_label.pack(pady=10)

    # Status Label
    status_label = ttk.Label(root, text="Ready", font=("Arial", 14), wraplength=400)  # Added wrap length
    status_label.pack(pady=10)

    # Button Frame
    button_frame = ttk.Frame(root)
    button_frame.pack()

    # Create the buttons using the custom style
    forward_button = ttk.Button(button_frame, text="Forward", command=move_forward, style="LargeButton.TButton")
    backward_button = ttk.Button(button_frame, text="Backward", command=move_backward, style="LargeButton.TButton")
    left_button = ttk.Button(button_frame, text="Left", command=turn_left, style="LargeButton.TButton")
    right_button = ttk.Button(button_frame, text="Right", command=turn_right, style="LargeButton.TButton")
    stop_button = ttk.Button(button_frame, text="Stop", command=stop_robot, style="SmallerButton.TButton")  # smaller

    # Grid layout for buttons
    forward_button.grid(row=0, column=1, padx=5, pady=5)
    backward_button.grid(row=2, column=1, padx=5, pady=5)
    left_button.grid(row=1, column=0, padx=5, pady=5)
    right_button.grid(row=1, column=2, padx=5, pady=5)
    stop_button.grid(row=1, column=1, padx=5, pady=5)  # center stop


def update_battery_display():
    """Updates the battery level display."""
    battery_level = get_battery_level()
    battery_label.config(text=f"Battery: {battery_level}%")
    # Reschedule the update after 5 seconds (5000 ms)
    root.after(ms=5000, func=update_battery_display)


def main():
    """Main function to create and run the application."""
    create_app()
    # Start the battery level update loop
    update_battery_display()
    root.mainloop()


if __name__ == "__main__":
    main()
