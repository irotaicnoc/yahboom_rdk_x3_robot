import time
import smbus
import threading

from oled import OLED
from joystick import Joystick
from SunriseRobotLib import SunriseRobot

import args
import utils
from ai_agent import AiAgent
from robot_head import RobotHead


def main_loop(**kwargs):
    parameters = args.import_args(
        yaml_path='/root/sunriseRobot/app_SunriseRobot/configs/main_thread_config.yaml',
        **kwargs,
    )
    robot_body = SunriseRobot()
    robot_head = RobotHead()

    task_joystick_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'verbose': False,
    }
    task_1 = threading.Thread(target=task_joystick, name='task_joystick', kwargs=task_joystick_kwargs)
    task_1.start()

    task_screen_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'verbose': parameters['verbose']
    }
    task_2 = threading.Thread(target=task_screen, name='task_screen', kwargs=task_screen_kwargs)
    task_2.start()

    task_ai_agent_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'verbose': parameters['verbose']
    }
    task_3 = threading.Thread(target=task_ai_agent, name='task_ai_agent', kwargs=task_ai_agent_kwargs)
    task_3.start()

#     TODO: ros2 as separate thread?


# USB wireless gamepad
def task_joystick(**kwargs):
    js = Joystick(**kwargs)
    while True:
        state = js.joystick_handle()
        if state != js.STATE_OK:
            if state == js.STATE_KEY_BREAK:
                break
            time.sleep(1)
            js.reconnect()


def task_ai_agent(**kwargs):
    try:
        ai_agent = AiAgent(**kwargs)
        robot_head = kwargs['robot_head']
        robot_head.robot_mode_list.append('autonomous_tracking')
        while True:
            ai_agent.autonomous_behavior()
    except Exception as e:
        print('Error: could not create AiAgent.')
        print(e)


# oled screen
def task_screen(**kwargs):
    try:
        oled_clear = False
        oled = OLED(clear=oled_clear, **kwargs)
        try:
            bus = smbus.SMBus(0)
            if not oled_clear:
                start = 1
                bus.write_byte_data(0x0d, 0x08, start)
                time.sleep(.05)
                effect = 2
                bus.write_byte_data(0x0d, 0x04, effect)
                time.sleep(.05)
        except:
            pass

        while True:
            state = oled.main_program()
            oled.clear(True)
            if state:
                del oled
                print("---OLED CLEARED!---")
                utils.close_rgb_fan(bus=bus)
                break
            time.sleep(1)
    except KeyboardInterrupt:
        del oled
        utils.close_rgb_fan(bus=bus)
        print("---Program closed!---")


if __name__ == '__main__':
    main_loop()
