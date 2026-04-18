#!/usr/bin/env python3
# coding=utf-8

import args
import utils
import global_constants as gc
from ros2.vr_controller_listener import ThreadedVrControllerListener


class MetaQuest3Controller(object):
    def __init__(self, controller_functions, controller_id: int = 1, verbose: int = 0):
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'meta_quest_3_controller.yaml', verbose=verbose)
        self.verbose = parameters['verbose']
        self.topic_name = parameters['topic_name']
        if self.verbose >= 1:
            print(f'VR Controller topic name: {self.topic_name}')
        self.controller_id = int(controller_id)
        self.controller_functions = controller_functions
        self._is_connected = False

        try:
            # Start the ROS2 listener in the background
            self.controller_listener = ThreadedVrControllerListener(topic_name=self.topic_name, verbose=self.verbose)
            self._is_connected = True
            self.controller_functions.connected(controller_id=self.controller_id)
        except Exception as e:
            self._is_connected = False
            # print(f'Failed to initialize VR Controller {self.controller_id}:\n\t{e}')

        # Thresholds to prevent joystick drift spam and convert analog triggers to buttons
        self.axis_deadzone = float(parameters['axis_deadzone'])
        self.trigger_threshold = float(parameters['trigger_threshold'])

        # State tracking to only send commands on state CHANGE (simulating events)
        self._prev_axes = [0.0] * 8
        self._prev_buttons = [0] * 6
        self._prev_triggers_as_buttons = [False] * 4  # L2 (grip), r2 (grip), L1 (index), R1 (index)

    def __del__(self):
        self._is_connected = False
        self.controller_listener.delete_listener()
        self.controller_functions.disconnected(controller_id=self.controller_id)
        if self.verbose >= 1:
            print(f'VR Controller {self.controller_id} closed')

    def _check_axis_change(self, current_val, prev_val):
        return abs(current_val - prev_val) > self.axis_deadzone

    def event_listener(self):
        """
        Call this continuously in your main robot loop instead of the PS2 event_listener.
        It processes the latest Joy message and fires functions only if inputs changed.
        """
        if not self._is_connected:
            return gc.STATE_DISCONNECT

        axes, buttons = self.controller_listener.get_axes_and_buttons()

        if axes is None or buttons is None:
            # No data received yet, or connection lost
            return gc.STATE_OK

        try:
            # --- PROCESS BUTTONS ---
            # Buttons 3 (Y) and 6 (Menu) are used by the VR app
            # A (South)
            if buttons[0] != self._prev_buttons[0]:
                self.controller_functions.button_south(bool(buttons[0]))
                self._prev_buttons[0] = buttons[0]
            # B (East)
            if buttons[1] != self._prev_buttons[1]:
                self.controller_functions.button_east(bool(buttons[1]))
                self._prev_buttons[1] = buttons[1]
            # X (West)
            if buttons[2] != self._prev_buttons[2]:
                self.controller_functions.button_west(bool(buttons[2]))
                self._prev_buttons[2] = buttons[2]
            # Y (North)
            # if buttons[3] != self._prev_buttons[3]:
            #     self.controller_functions.button_north(bool(buttons[3]))
            #     self._prev_buttons[3] = buttons[3]
            # Left Stick Click
            if buttons[4] != self._prev_buttons[4]:
                self.controller_functions.button_select(bool(buttons[4]))
                self._prev_buttons[4] = buttons[4]
            # Right Stick Click
            if buttons[5] != self._prev_buttons[5]:
                self.controller_functions.button_start(bool(buttons[5]))
                self._prev_buttons[5] = buttons[5]
            # Menu Button
            # if buttons[6] != self._prev_buttons[6]:
            #     pass

            # --- PROCESS TRIGGERS AS BUTTONS ---
            # Quest triggers are analog. With threshold > 0.5 they act as L1/R1/L2/R2
            l2_pressed = axes[3] > self.trigger_threshold  # Left Grip
            if l2_pressed != self._prev_triggers_as_buttons[0]:
                self.controller_functions.button_l2(l2_pressed)
                self._prev_triggers_as_buttons[0] = l2_pressed

            r2_pressed = axes[7] > self.trigger_threshold  # Right Grip
            if r2_pressed != self._prev_triggers_as_buttons[1]:
                self.controller_functions.button_r2(r2_pressed)
                self._prev_triggers_as_buttons[1] = r2_pressed

            l1_pressed = axes[2] > self.trigger_threshold  # Left Index
            if l1_pressed != self._prev_triggers_as_buttons[2]:
                self.controller_functions.button_l1(l1_pressed, from_vr=True)
                self._prev_triggers_as_buttons[2] = l1_pressed

            r1_pressed = axes[6] > self.trigger_threshold  # Right Index
            if r1_pressed != self._prev_triggers_as_buttons[3]:
                self.controller_functions.button_r1(r1_pressed, from_vr=True)
                self._prev_triggers_as_buttons[3] = r1_pressed

            # --- PROCESS AXES ---
            # they are already thresholded on the VR side
            if self._check_axis_change(axes[0], self._prev_axes[0]):
                self.controller_functions.axis_left_x(axes[0])
                self._prev_axes[0] = axes[0]

            if self._check_axis_change(axes[1], self._prev_axes[1]):
                self.controller_functions.axis_left_y(axes[1])
                self._prev_axes[1] = axes[1]

            if self._check_axis_change(axes[4], self._prev_axes[4]):
                self.controller_functions.axis_right_x(axes[4])
                self._prev_axes[4] = axes[4]

            if self._check_axis_change(axes[5], self._prev_axes[5]):
                self.controller_functions.axis_right_y(axes[5])
                self._prev_axes[5] = axes[5]

            return gc.STATE_OK

        except Exception as e:
            utils.print_exception(exception=e, message='VR Controller processing error')
            return gc.STATE_DISCONNECT
