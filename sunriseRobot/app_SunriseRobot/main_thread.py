import time
import threading

from oled import OLED
from joystick import Joystick
from robot_body import RobotBody

import args
from light import Light
from ai_agent import AiAgent
import global_constants as gc
from robot_head import RobotHead
from gpio_pin_control import GpioLed


def main_loop(**kwargs):
    parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'main_thread_config.yaml', **kwargs)

    robot_body = RobotBody(com=parameters['com'], baud_rate=parameters['baud_rate'], verbose=parameters['verbose'])
    robot_body.create_receive_threading()
    robot_head = RobotHead()
    internal_light = Light(verbose=parameters['verbose'])
    gpio_led = GpioLed()

    joystick_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'internal_light': internal_light,
        'gpio_led': gpio_led,
        'verbose': parameters['verbose'],
    }
    thread_joystick = threading.Thread(target=task_joystick, name='task_joystick', kwargs=joystick_kwargs)
    thread_joystick.start()

    screen_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'verbose': parameters['verbose'],
    }
    thread_screen = threading.Thread(target=task_screen, name='task_screen', kwargs=screen_kwargs)
    thread_screen.start()

    ai_agent_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'gpio_led': gpio_led,
        'verbose': parameters['verbose'],
    }
    thread_ai_agent = threading.Thread(target=task_ai_agent, name='task_ai_agent', kwargs=ai_agent_kwargs)
    thread_ai_agent.start()

    # TODO: ros2 as separate thread?

    # notify the robot is ready
    robot_body.set_beep(50)


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
        print('AiAgent Error:')
        print(e)
        print(e.__traceback__)


# oled screen
def task_screen(**kwargs):
    try:
        oled = OLED(clear=False, **kwargs)
        while True:
            state = oled.main_program()
            oled.clear(True)
            if state:
                del oled
                print('---OLED CLEARED!---')
                break
            time.sleep(1)
    except KeyboardInterrupt:
        del oled
        print('---Program closed!---')


if __name__ == '__main__':
    main_loop()
