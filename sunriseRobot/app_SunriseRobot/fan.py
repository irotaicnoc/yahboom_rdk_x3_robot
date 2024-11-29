import time

import global_constants as gc
from robot_head import RobotHead


class Fan:
    def __init__(self, robot_head: RobotHead):
        self.robot_head = robot_head
        self.current_state = gc.FAN_STOP_CMD
        self.stop()

    def stop(self):
        if self.robot_head.verbose >= 2:
            print('Stopping fan...', end='')
        self.robot_head.bus.write_byte_data(gc.BUS_ARG_1, gc.BUS_ARG_2_FAN_STATE, gc.FAN_STOP_CMD)
        self.current_state = gc.FAN_STOP_CMD
        time.sleep(.05)
        if self.robot_head.verbose >= 2:
            print('Done.')

    def start(self):
        if self.robot_head.verbose >= 2:
            print('Starting fan...', end='')
        self.robot_head.bus.write_byte_data(gc.BUS_ARG_1, gc.BUS_ARG_2_FAN_STATE, gc.FAN_START_CMD)
        self.current_state = gc.FAN_START_CMD
        time.sleep(.05)
        if self.robot_head.verbose >= 2:
            print('Done.')

    def control_behavior(self):
        if self.robot_head.fan_state != self.current_state:
            if self.robot_head.fan_state == gc.FAN_START_CMD:
                self.start()
            elif self.robot_head.fan_state == gc.FAN_STOP_CMD:
                self.stop()

        time.sleep(5)

    def __del__(self):
        self.stop()
