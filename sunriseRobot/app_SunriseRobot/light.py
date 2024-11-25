import time

import global_constants as gc
from robot_head import RobotHead


class Light:
    def __init__(self, robot_head: RobotHead):
        self.current_state = gc.LIGHT_STOP_CMD
        self.robot_head = robot_head
        self.stop()

    def stop(self):
        self.robot_head.bus.write_byte_data(gc.BUS_ARG_1, gc.BUS_ARG_2_LIGHT_STATE, gc.LIGHT_STOP_CMD)
        self.current_state = gc.LIGHT_STOP_CMD
        time.sleep(.05)

    def set_state(self, state: int):
        if state == gc.LIGHT_STOP_CMD:
            self.stop()
        else:
            self.robot_head.bus.write_byte_data(gc.BUS_ARG_1, gc.BUS_ARG_2_LIGHT_MODE, state)
            self.current_state = state
            time.sleep(.05)

    def control_behavior(self):
        if self.robot_head.light_state != self.current_state:
            self.set_state(self.robot_head.light_state)

        time.sleep(1)

    def __del__(self):
        self.stop()
