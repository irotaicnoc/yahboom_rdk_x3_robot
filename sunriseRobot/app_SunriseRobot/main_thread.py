import time
import threading

from oled import OLED
from joystick import Joystick
from SunriseRobotLib import SunriseRobot

import args
from fan import Fan
from light import Light
from ai_agent import AiAgent
from robot_head import RobotHead


def main_loop(**kwargs):
    parameters = args.import_args(
        yaml_path='/root/sunriseRobot/app_SunriseRobot/configs/main_thread_config.yaml',
        **kwargs,
    )

    robot_body = SunriseRobot(com="/dev/ttyUSB0", debug=False)
    robot_body.create_receive_threading()

    robot_head = RobotHead()
    if parameters['verbose']:
        print(f'robot_mode_list: {robot_head.robot_mode_list}')
        print(f'robot_mode: {robot_head.robot_mode}')
        print(f'tracking_target_list: {robot_head.tracking_target_list}')
        print(f'selected_target: {robot_head.tracking_target_list[robot_head.tracking_target_pos]}')
        print(f'speed_coefficient: {robot_head.speed_coefficient}')

    task_1_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'verbose': False,
    }
    task_1 = threading.Thread(target=task_joystick, name='task_joystick', kwargs=task_1_kwargs)
    task_1.start()

    task_2_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'verbose': parameters['verbose'],
    }
    task_2 = threading.Thread(target=task_screen, name='task_screen', kwargs=task_2_kwargs)
    task_2.start()

    task_3_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'verbose': parameters['verbose'],
    }
    task_3 = threading.Thread(target=task_ai_agent, name='task_ai_agent', kwargs=task_3_kwargs)
    task_3.start()

    fan_kwargs = {
        'robot_head': robot_head,
    }
    thread_fan = threading.Thread(target=task_fan, name='task_fan', kwargs=fan_kwargs)
    thread_fan.start()

    light_kwargs = {
        'robot_head': robot_head,
    }
    thread_light = threading.Thread(target=task_light, name='task_light', kwargs=light_kwargs)
    thread_light.start()

    # TODO: ros2 as separate thread?


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
        oled = OLED(clear=False, **kwargs)
        while True:
            state = oled.main_program()
            oled.clear(True)
            if state:
                del oled
                print("---OLED CLEARED!---")
                break
            time.sleep(1)
    except KeyboardInterrupt:
        del oled
        print("---Program closed!---")


def task_fan(**kwargs):
    fan = Fan(**kwargs)
    time.sleep(.2)
    while True:
        fan.control_behavior()


def task_light(**kwargs):
    light = Light(**kwargs)
    time.sleep(.2)
    while True:
        light.control_behavior()


if __name__ == '__main__':
    main_loop()
