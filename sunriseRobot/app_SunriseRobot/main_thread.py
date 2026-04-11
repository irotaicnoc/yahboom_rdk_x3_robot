import time
import warnings
import threading
import Hobot.GPIO as GPIO

import args
import utils
import global_constants as gc
from robot_body import RobotBody
from robot_head import RobotHead
from gpio.led_3_pin import Led3Pin
from gpio.button_2_pin import Button2Pin
from physical_accessories.arm import Arm
from physical_accessories.oled import Oled
from physical_accessories.light import Light
from controllers.ps2_controller import PS2Controller
from controllers.controller_loop import ControllerLoop
from controllers.controller_interface import ControllerFunctions
from controllers.meta_quest_3_controller import MetaQuest3Controller
from ros2.vr_audio_publisher import VrAudioPublisher
from ros2.vr_audio_subscriber import VrAudioSubscriber
from ethernet_connection.ethernet_server import EthernetServer


def main_loop(**kwargs):
    parameters = args.import_args(
        yaml_path=gc.CONFIG_FOLDER_PATH + 'main_thread.yaml',
        read_from_command_line=True,
        **kwargs,
    )
    if parameters['gui_mode']:
        print('Running in GUI mode')
    # else:
    #     parameters['verbose'] = 0  # disable verbose output in non-GUI mode
    robot_body = RobotBody(com=parameters['com'], baud_rate=parameters['baud_rate'], verbose=parameters['verbose'])
    robot_body.create_receive_threading()
    time.sleep(0.2)  # wait for the robot body to initialize

    # LIGHTS
    internal_light = Light(verbose=parameters['verbose'])
    led_3_pin = Led3Pin(red_power_cable=gc.RED_CABLE_01, green_power_cable=gc.GREEN_CABLE_01)

    robot_head = RobotHead(
        robot_body=robot_body,
        internal_light=internal_light,
        led_3_pin=led_3_pin,
        gui_mode=parameters['gui_mode'],
        verbose=parameters['verbose'],
    )

    # PHYSICAL GPIO BUTTONS ON THE ROBOT
    # 2-PIN BUTTON (blue-black cables)
    #    with short click: toggle hotspot
    #    with long click: toggle ros2 vr connection
    button_2_pin_bb_listener_kwargs = {
            'control_cable': gc.BLUE_CABLE_01,
            # 'callback_short_click': robot_head.toggle_hotspot,
            'callback_short_click': utils.test_button_bb_short_click,
            # 'callback_long_click': robot_head.toggle_ros2_vr_connection,
            'callback_long_click': utils.test_button_bb_long_click,
            'button_press_required_time': robot_head.button_press_required_time,
    }
    thread_button_2_pin_bb = threading.Thread(
        target=task_button_2_pin_listener,
        name='task_button_2_pin_bb_listener',
        kwargs=button_2_pin_bb_listener_kwargs,
    )
    thread_button_2_pin_bb.start()
    # 2-PIN BUTTON (red-black cables)
    #    with short click: next mode
    #    with long click: toggle GUI mode
    button_2_pin_rb_listener_kwargs = {
            'control_cable': gc.RED_CABLE_02,
            # 'callback_short_click': robot_head.next_mode,
            'callback_short_click': utils.test_button_rb_short_click,
            # 'callback_long_click': robot_head.toggle_gui_mode,
            'callback_long_click': utils.test_button_rb_long_click,
            'button_press_required_time': robot_head.button_press_required_time,
    }
    thread_button_2_pin_rb = threading.Thread(
        target=task_button_2_pin_listener,
        name='task_button_2_pin_rb_listener',
        kwargs=button_2_pin_rb_listener_kwargs,
    )
    thread_button_2_pin_rb.start()

    # ARM
    try:
        arm = Arm(robot_head=robot_head, robot_body=robot_body, verbose=parameters['verbose'])
    except Exception as e:
        utils.print_exception(exception=e, message='Arm error')
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

    ps2_controller_kwargs = {
        'controller_id': parameters['controller_id'],
        'robot_head': robot_head,
        'robot_body': robot_body,
        'arm': arm,
        'verbose': parameters['verbose'],
    }
    thread_ps2_controller = threading.Thread(
        target=task_ps2_controller,
        name='task_ps2_controller',
        kwargs=ps2_controller_kwargs,
    )
    thread_ps2_controller.start()

    vr_controller_kwargs = {
        'controller_id': parameters['controller_id'] + 1,
        'robot_head': robot_head,
        'robot_body': robot_body,
        'arm': arm,
        'verbose': parameters['verbose'],
    }
    thread_vr_controller = threading.Thread(
        target=task_vr_controller,
        name='task_vr_controller',
        kwargs=vr_controller_kwargs,
    )
    thread_vr_controller.start()

    audio_from_vr_kwargs = {
        'robot_head': robot_head,
        'verbose': parameters['verbose'],
    }
    thread_audio_from_vr = threading.Thread(
        target=task_audio_from_vr,
        name='task_audio_from_vr',
        kwargs=audio_from_vr_kwargs,
    )
    thread_audio_from_vr.start()

    audio_from_robot_kwargs = {
        'robot_head': robot_head,
        'verbose': parameters['verbose'],
    }
    thread_audio_from_robot = threading.Thread(
        target=task_audio_from_robot,
        name='task_audio_from_robot',
        kwargs=audio_from_robot_kwargs,
    )
    thread_audio_from_robot.start()

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

    # execute the start callbacks for the initial robot mode
    if robot_head.robot_mode in robot_head.mode_start_callbacks:
        for callback in robot_head.mode_start_callbacks[robot_head.robot_mode]:
            callback()
    # execute the start callbacks for the initial robot sub mode
    if robot_head.robot_sub_mode is not None:
        if robot_head.robot_sub_mode in robot_head.sub_mode_start_callbacks:
            for callback in robot_head.sub_mode_start_callbacks[robot_head.robot_sub_mode]:
                callback()

    try:
        ethernet_server = EthernetServer(
            robot_head=robot_head,
            robot_body=robot_body,
            arm=arm,
            light=internal_light,
            verbose=parameters['verbose'],
        )
        ethernet_server.start()
    except Exception as e:
        utils.print_exception(exception=e, message='Ethernet server error')

    # notify the robot is ready
    robot_body.set_beep(gc.SHORT_BEEP)


# USB wireless gamepad
def task_ps2_controller(**kwargs):
    try:
        controller_functions = ControllerFunctions(
            robot_head=kwargs['robot_head'],
            robot_body=kwargs['robot_body'],
            arm=kwargs['arm'],
            verbose=kwargs['verbose'],
        )
        ps2_controller = PS2Controller(
            controller_functions=controller_functions,
            controller_id=kwargs['controller_id'],
            verbose=kwargs['verbose'],
        )
        while True:
            state = ps2_controller.event_listener()
            time.sleep(0.01)
            if state != gc.STATE_OK:
                if state == gc.STATE_KEY_BREAK:
                    break
                time.sleep(1)
                ps2_controller.reconnect()

    except Exception as e:
        utils.print_exception(exception=e, message='PS2 Controller error')


# VR controllers
def task_vr_controller(**kwargs):
    try:
        controller_functions = ControllerFunctions(
            robot_head=kwargs['robot_head'],
            robot_body=kwargs['robot_body'],
            arm=kwargs['arm'],
            verbose=kwargs['verbose'],
        )
        meta_quest_3_controller = None
        while True:
            if meta_quest_3_controller is None:
                if kwargs['robot_head'].ros2_vr_connection_status == 'active':
                    meta_quest_3_controller = MetaQuest3Controller(
                        controller_functions=controller_functions,
                        # differentiate this VR controller from the main one
                        controller_id=kwargs['controller_id'],
                        verbose=kwargs['verbose'],
                    )
                    time.sleep(0.5)
            else:  # meta_quest_3_controller is not None
                if kwargs['robot_head'].ros2_vr_connection_status == 'active':
                    meta_quest_3_controller.event_listener()
                elif kwargs['robot_head'].ros2_vr_connection_status == 'inactive':
                    del meta_quest_3_controller
                    meta_quest_3_controller = None
                    time.sleep(0.5)
            time.sleep(0.01)

    except Exception as e:
        utils.print_exception(exception=e, message='VR Controller error')


def task_controller_loop(**kwargs):
    try:
        controller_loop = ControllerLoop(**kwargs)
        while True:
            controller_loop.update_robot_loop()
    except Exception as e:
        utils.print_exception(exception=e, message='Controller loop error')


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
        utils.print_exception(exception=e, message='Vision agent error')
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
#         utils.print_exception(exception=e, message='Sound agent error')
#         if gc.MODE_AUTONOMOUS_SOUND in robot_head.robot_mode_list:
#             robot_head.robot_mode_list.remove(gc.MODE_AUTONOMOUS_SOUND)
#             if robot_head.robot_mode == gc.MODE_AUTONOMOUS_SOUND:
#                 robot_head.robot_mode = robot_head.robot_mode_list[0]
#                 if robot_head.robot_sub_mode_dict[robot_head.robot_mode] is not None:
#                     robot_head.robot_sub_mode = robot_head.robot_sub_mode_dict[robot_head.robot_mode][0]


# Audio from VR
def task_audio_from_vr(**kwargs):
    try:
        audio_from_vr_parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'vr_audio_subscriber.yaml',
            read_from_command_line=False,
            **kwargs,
        )
        meta_quest_3_audio_receiver = None
        while True:
            if meta_quest_3_audio_receiver is None and kwargs['robot_head'].ros2_vr_connection_status == 'active':
                meta_quest_3_audio_receiver = VrAudioSubscriber(**audio_from_vr_parameters)
                time.sleep(0.5)
            elif (meta_quest_3_audio_receiver is not None
                  and kwargs['robot_head'].ros2_vr_connection_status == 'inactive'):
                del meta_quest_3_audio_receiver
                meta_quest_3_audio_receiver = None
                time.sleep(0.5)
            else:
                time.sleep(0.05)

    except Exception as e:
        utils.print_exception(exception=e, message='Audio from VR error')


# Audio from Robot
def task_audio_from_robot(**kwargs):
    try:
        audio_from_robot_parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'vr_audio_publisher.yaml',
            read_from_command_line=False,
            **kwargs,
        )
        meta_quest_3_audio_sender = None
        while True:
            if meta_quest_3_audio_sender is None and kwargs['robot_head'].ros2_vr_connection_status == 'active':
                meta_quest_3_audio_sender = VrAudioPublisher(**audio_from_robot_parameters)
                time.sleep(0.5)
            elif (meta_quest_3_audio_sender is not None
                  and kwargs['robot_head'].ros2_vr_connection_status == 'inactive'):
                del meta_quest_3_audio_sender
                meta_quest_3_audio_sender = None
                time.sleep(0.5)
            else:
                time.sleep(0.05)

    except Exception as e:
        utils.print_exception(exception=e, message='Audio from Robot error')


def task_button_2_pin_listener(**kwargs):
    try:
        button_2_pin = Button2Pin(**kwargs)
        while True:
            button_2_pin.press_listener()
    except Exception as e:
        utils.print_exception(exception=e, message='Button listener error')


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
    except Exception as e:
        utils.print_exception(exception=e, message='Oled error')
        del oled


if __name__ == '__main__':
    try:
        main_loop()
    except:
        GPIO.cleanup()
