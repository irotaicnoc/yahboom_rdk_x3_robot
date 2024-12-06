import time
import warnings

from robot_body import RobotBody
from hobot_vio import libsrcampy as camera_lib

import args
import utils
import global_constants as gc
from robot_head import RobotHead
from detector import YoloDetector
from gpio_pin_control import GpioLed


class AiAgent(object):
    def __init__(self, robot_body: RobotBody, robot_head: RobotHead, gpio_led: GpioLed, **kwargs):
        # general initialization
        self.robot_body = robot_body
        self.robot_head = robot_head
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'ai_agent_config.yaml',
            **kwargs,
        )
        self.verbose = parameters['verbose']
        self.agent_active = False
        self.think_steps_if_no_target = parameters['think_steps_if_no_target']
        self.no_target_counter = 0

        # camera initialization
        self.camera_is_open = -1
        self.camera = camera_lib.Camera()
        self.video_capture_kwargs = parameters['camera_kwargs']['video_capture_kwargs']
        self.frame_width = self.video_capture_kwargs['width']
        self.frame_height = self.video_capture_kwargs['height']

        # yolo detector initialization
        self.detector = YoloDetector(
            robot_head=robot_head,
            camera_image_size=(self.frame_width, self.frame_height),
            verbose=self.verbose,
        )
        self.save_images = parameters['save_images']

        # motion initialization
        self.steer_threshold = parameters['steer_threshold']
        self.angular_speed_range = parameters['angular_speed_range']
        self.speed_x = 0
        self.speed_z = 0

        # gpio led
        self.gpio_led = gpio_led
        self.use_gpio_led = parameters['use_gpio_led']

    def set_zero_speed(self):
        self.speed_x = 0
        self.speed_z = 0

    def deactivate_agent(self):
        if self.verbose >= 1:
            print('Deactivating autonomous agent...')
        self.set_zero_speed()
        self.robot_body.set_car_motion(self.speed_x, 0, self.speed_z)
        self.agent_active = False
        if self.camera_is_open == 0:
            self.camera_is_open = -1
            self.camera.close_cam()
            if self.verbose >= 2:
                print('Camera closed.')

        # turn off gpio led
        if self.use_gpio_led:
            self.gpio_led.set_color('off')

    def activate_agent(self, video_capture_kwargs=None):
        if self.verbose >= 1:
            print('Activating autonomous agent...')
        self.set_zero_speed()
        self.robot_body.set_car_motion(self.speed_x, 0, self.speed_z)
        self.camera_is_open = -1
        if video_capture_kwargs is None:
            self.camera_is_open = self.camera.open_cam(**self.video_capture_kwargs)
        else:
            self.camera_is_open = self.camera.open_cam(**video_capture_kwargs)

        if self.camera_is_open == 0:
            self.agent_active = True
            self.gpio_led.set_color('off')
            if self.verbose >= 2:
                print('Camera opened correctly.')
        else:
            warnings.warn('Failed to open camera.')
            warnings.warn('Impossible to run autonomous agent.')
            self.agent_active = False

    def autonomous_behavior(self):
        if self.robot_head.robot_mode == 'autonomous_tracking':
            if self.agent_active:
                self.detect_and_move()
            else:
                self.activate_agent()
                self.detect_and_move()
        else:
            if self.agent_active:
                self.deactivate_agent()
                time.sleep(2)
            else:
                time.sleep(2)

    # stop -> observe -> think -> move for n seconds -> repeat until interrupted
    def detect_and_move(self) -> None:
        # show thinking light (red)
        if self.use_gpio_led:
            self.gpio_led.set_color('red')
        if self.verbose >= 2:
            start_thinking = time.time()
        self.set_zero_speed()
        self.robot_body.set_car_motion(self.speed_x, 0, self.speed_z)
        move_duration = 0.6

        frame = self.camera.get_img(2)
        if frame is None:
            if self.verbose >= 1:
                print('Frame is None.')
            time.sleep(0.5)
            return

        target_info = self.detector.find_target(
            frame=frame,
            target_name=self.robot_head.tracking_target_list[self.robot_head.tracking_target_pos],
            save=self.save_images,
        )
        # target_info = {
        #     'num_targets': int,
        #     'highest_confidence': float [0, 1],
        #     'distance_from_center_x': float [-1, 1],
        #     'distance_from_center_y': float [-1, 1],
        # }
        if target_info['num_targets'] > 0:
            # show target-found light (green)
            if self.use_gpio_led:
                self.gpio_led.set_color('green')
            self.no_target_counter = 0
            distance_from_center_x = target_info['distance_from_center_x']
            # print(f'target x: {distance_from_center_x}')
            # only steer to the target if its center is more than
            # self.steer_threshold distant from the current forward direction
            # otherwise move forward
            if self.verbose >= 2:
                print(f'X distance from img center: {distance_from_center_x}')
            if abs(distance_from_center_x) > self.steer_threshold:
                self.speed_x = 0
                self.speed_z = utils.x_displacement_to_angular_speed(
                    x_distance_from_img_center=distance_from_center_x,
                    steer_threshold=self.steer_threshold,
                    angular_speed_range=self.angular_speed_range,
                )
                if self.verbose >= 2:
                    print(f'Steer: {self.speed_z}')
            else:
                self.speed_x = self.robot_head.speed_coefficient
                if self.verbose >= 2:
                    print(f'Forward: {self.speed_x}')
                self.speed_z = 0
                # move_duration = 0.6
        else:
            # show target-not-found/searching light (red_and_green)
            if self.use_gpio_led:
                self.gpio_led.set_color('red_and_green')
            if self.verbose >= 2:
                print('Searching...')
            # TODO: very slowly rotate by 360° degree
            self.speed_x = 0
            if self.no_target_counter < self.think_steps_if_no_target:
                self.no_target_counter += 1
                if self.verbose >= 2:
                    print('\nThink more before moving')
                time.sleep(0.1)
                return
            else:
                self.speed_z = self.robot_head.speed_coefficient * 5
                self.no_target_counter = 0
            if self.verbose >= 2:
                print(f'Steer: {self.speed_z}')
        if self.verbose >= 2:
            stop_thinking = time.time()
            print(f'thinking time: {round(stop_thinking - start_thinking, 3)}')

        # start_moving = time.time()
        self.robot_body.set_car_motion(self.speed_x, 0, self.speed_z)
        time.sleep(move_duration)
        # stop_moving = time.time()
        # print(f'moving time AI: {round(stop_moving - start_moving, 3)}')

    def __del__(self):
        self.deactivate_agent()
