import time
import warnings
import threading

import args
import global_constants as gc
from robot_body import RobotBody
from robot_head import RobotHead
from physical_accessories.arm import Arm
from physical_accessories.oled import Oled
from physical_accessories.light import Light
from controllers.ps2_controller import PS2Controller
from controllers.controller_loop import ControllerLoop
from physical_accessories.gpio_pin_control import GpioLed
from controllers.controller_interface import ControllerFunctions


def main_loop(**kwargs):
    parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'main_thread.yaml', **kwargs)
    robot_body = RobotBody(com=parameters['com'], baud_rate=parameters['baud_rate'], verbose=parameters['verbose'])
    robot_body.create_receive_threading()

    # LIGHTS
    internal_light = Light(verbose=parameters['verbose'])
    gpio_led = GpioLed()

    robot_head = RobotHead(
        internal_light=internal_light,
        gpio_led=gpio_led,
        verbose=parameters['verbose'],
    )

    # ARM
    try:
        arm = Arm(robot_head=robot_head, robot_body=robot_body, verbose=parameters['verbose'])
    except Exception as e:
        print('Arm error:')
        print(e)
        print(e.__traceback__)
        arm = None

    # CONTROLLER
    controller_loop_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'arm': arm,
        'verbose': parameters['verbose'],
    }
    thread_controller_loop = threading.Thread(
        target=task_controller_loop,
        name='task_controller_loop',
        kwargs=controller_loop_kwargs,
    )
    thread_controller_loop.start()
    controller_kwargs = {
        'controller_id': parameters['controller_id'],
        'robot_head': robot_head,
        'robot_body': robot_body,
        'arm': arm,
        'verbose': parameters['verbose'],
    }
    thread_controller = threading.Thread(target=task_controller, name='task_controller', kwargs=controller_kwargs)
    thread_controller.start()

    # Oled SCREEN
    screen_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'verbose': parameters['verbose'],
    }
    # "daemon = True" means that when this is the only thread running, (or when only other daemonic threads remain)
    # the containing thread (the main) will exit. The oled screen is daemonic, if there are no more controller and/or
    # vision_agent left, then the main can stop.
    thread_screen = threading.Thread(target=task_screen, name='task_screen', kwargs=screen_kwargs, daemon=True)
    thread_screen.start()

    # VISION AGENT
    vision_agent_kwargs = {
        'robot_body': robot_body,
        'robot_head': robot_head,
        'camera_type': parameters['camera_type'],
        'verbose': parameters['verbose'],
    }
    thread_vision_agent = threading.Thread(
        target=task_vision_agent,
        name='task_vision_agent',
        kwargs=vision_agent_kwargs,
    )
    thread_vision_agent.start()

    # SOUND AGENT
    # sound_agent_kwargs = {
    #     'robot_body': robot_body,
    #     'robot_head': robot_head,
    #     'verbose': parameters['verbose'],
    # }
    # thread_sound_agent = threading.Thread(
    #     target=task_sound_agent,
    #     name='task_sound_agent',
    #     kwargs=sound_agent_kwargs,
    # )
    # thread_sound_agent.start()

    # notify the robot is ready
    robot_body.set_beep(50)


# USB wireless gamepad
def task_controller(**kwargs):
    try:
        controller_functions = ControllerFunctions(
            robot_head=kwargs['robot_head'],
            robot_body=kwargs['robot_body'],
            arm=kwargs['arm'],
            verbose=kwargs['verbose'],
        )
        ps2_controller = PS2Controller(controller_functions=controller_functions, controller_id=kwargs['controller_id'])
        while True:
            state = ps2_controller.event_listener()
            if state != ps2_controller.STATE_OK:
                if state == ps2_controller.STATE_KEY_BREAK:
                    break
                time.sleep(1)
                ps2_controller.reconnect()
    except Exception as e:
        print('Controller error:')
        print(e)
        print(e.__traceback__)


def task_controller_loop(**kwargs):
    try:
        controller_loop = ControllerLoop(**kwargs)
        while True:
            controller_loop.update_robot_loop()
    except Exception as e:
        print('Controller loop error:')
        print(e)
        print(e.__traceback__)


def task_vision_agent(**kwargs):
    robot_head = kwargs['robot_head']
    try:
        robot_head.robot_mode_list.append(gc.MODE_AUTONOMOUS_VISION)
        if kwargs['camera_type'] == 'internal':
            from vision.vision_agent import VisionAgent
        elif kwargs['camera_type'] == 'usb_v1':
            try:
                from vision.vision_agent_usb_camera_v1 import VisionAgent
            except:
                warnings.warn(f'Could not find camera {kwargs["camera_type"]}. Switching to internal camera...')
                kwargs['camera_type'] = 'internal'
                from vision.vision_agent import VisionAgent
        elif kwargs['camera_type'] == 'usb_v2':
            try:
                from vision.vision_agent_usb_camera_v2 import VisionAgent
            except:
                warnings.warn(f'Could not find camera {kwargs["camera_type"]}. Switching to internal camera...')
                kwargs['camera_type'] = 'internal'
                from vision.vision_agent import VisionAgent
        else:
            raise ValueError(f'Unknown camera_type: {kwargs["camera_type"]}')
        vision_agent = VisionAgent(**kwargs)
        while True:
            vision_agent.autonomous_behavior()
    except Exception as e:
        print('Vision agent error:')
        print(e)
        print(e.__traceback__)
        if gc.MODE_AUTONOMOUS_VISION in robot_head.robot_mode_list:
            robot_head.robot_mode_list.remove(gc.MODE_AUTONOMOUS_VISION)
            if robot_head.robot_mode == gc.MODE_AUTONOMOUS_VISION:
                robot_head.robot_mode = robot_head.robot_mode_list[0]
                if robot_head.robot_sub_mode_dict[robot_head.robot_mode] is not None:
                    robot_head.robot_sub_mode = robot_head.robot_sub_mode_dict[robot_head.robot_mode][0]


# def task_sound_agent(**kwargs):
#     robot_head = kwargs['robot_head']
#     try:
#         robot_head.robot_mode_list.append(gc.MODE_AUTONOMOUS_SOUND)
#         sound_agent = SoundAgent(**kwargs)
#         while True:
#             sound_agent.autonomous_behavior()
#     except Exception as e:
#         print('Sound agent error:')
#         print(e)
#         print(e.__traceback__)
#         if gc.MODE_AUTONOMOUS_SOUND in robot_head.robot_mode_list:
#             robot_head.robot_mode_list.remove(gc.MODE_AUTONOMOUS_SOUND)
#             if robot_head.robot_mode == gc.MODE_AUTONOMOUS_SOUND:
#                 robot_head.robot_mode = robot_head.robot_mode_list[0]
#                 if robot_head.robot_sub_mode_dict[robot_head.robot_mode] is not None:
#                     robot_head.robot_sub_mode = robot_head.robot_sub_mode_dict[robot_head.robot_mode][0]


# oled screen
def task_screen(**kwargs):
    try:
        oled = Oled(clear=False, **kwargs)
        while True:
            state = oled.main_program()
            oled.clear(refresh=True)
            if not state:
                del oled
                warnings.warn('Oled error. Oled deactivated')
                break
            print('Oled cleared')
            time.sleep(2)
    except KeyboardInterrupt as e:
        del oled
        print('Oled error:')
        print(e)
        print(e.__traceback__)


if __name__ == '__main__':
    main_loop()
