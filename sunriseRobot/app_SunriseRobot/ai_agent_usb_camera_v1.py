import cv2
import time
import warnings

from robot_body import RobotBody

import args
import utils
import global_constants as gc
from robot_head import RobotHead
from gpio_pin_control import GpioLed
from detector_usb_camera_v1 import YoloDetector


class AiAgent(object):
    def __init__(self, robot_body: RobotBody, robot_head: RobotHead, gpio_led: GpioLed, **kwargs):
        # general initialization
        self.robot_body = robot_body
        self.robot_head = robot_head
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'ai_agent_usb_camera_v1.yaml',
            **kwargs,
        )
        self.verbose = parameters['verbose']
        self.agent_active = False
        self.think_steps_if_no_target = parameters['think_steps_if_no_target']
        self.no_target_counter = 0

        # camera initialization
        self.camera = None
        self.camera_is_open = False
        self.camera_index = parameters['camera_kwargs']['index']
        self.frame_width = parameters['camera_kwargs']['width']
        self.frame_height = parameters['camera_kwargs']['height']
        self.frame_per_second = parameters['camera_kwargs']['fps']
        self.buffer_size = parameters['camera_kwargs']['buffer_size']

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
        if self.camera_is_open:
            self.camera_is_open = False
            self.camera.release()
            if self.verbose >= 2:
                print('Camera closed.')

        # turn off gpio led
        if self.use_gpio_led:
            self.gpio_led.set_color('off')

    def activate_agent(self):
        if self.verbose >= 1:
            print('Activating autonomous agent...')
        self.set_zero_speed()
        self.robot_body.set_car_motion(self.speed_x, 0, self.speed_z)

        self.camera = cv2.VideoCapture(self.camera_index)
        self.camera_is_open = self.camera.isOpened()
        if self.camera_is_open:
            self.camera.set(propId=cv2.CAP_PROP_FRAME_WIDTH, value=self.frame_width)
            self.camera.set(propId=cv2.CAP_PROP_FRAME_HEIGHT, value=self.frame_height)
            self.camera.set(propId=cv2.CAP_PROP_FPS, value=self.frame_per_second)
            self.camera.set(propId=cv2.CAP_PROP_BUFFERSIZE, value=self.buffer_size)
            self.agent_active = True
            self.gpio_led.set_color('off')
            if self.verbose >= 2:
                print('Camera opened correctly.')
                frame_width = int(self.camera.get(propId=cv2.CAP_PROP_FRAME_WIDTH))
                frame_height = int(self.camera.get(propId=cv2.CAP_PROP_FRAME_HEIGHT))
                fps = int(self.camera.get(propId=cv2.CAP_PROP_FPS))
                print(f'camera index: {self.camera_index}')
                print(f'width: {frame_width}, height: {frame_height}')
                print(f'fps: {fps}, buffer size: {self.buffer_size}')
        else:
            warnings.warn(f'Failed to open camera {self.camera_index}.')
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
        move_duration = 0.5

        # remove the old frame from the buffer. "grab" is faster that "read".
        self.camera.grab()
        success, frame = self.camera.read()
        if not success:
            if self.verbose >= 1:
                print('Could not read frame.')
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
        if self.verbose >= 2:
            print(f'num_targets: {target_info["num_targets"]}')
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

            self.speed_x = 0
            if self.no_target_counter < self.think_steps_if_no_target:
                self.no_target_counter += 1
                if self.verbose >= 2:
                    print('\nThink more before moving')
                time.sleep(0.1)
                return
            else:
                if self.verbose >= 2:
                    print('Searching...')
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
