import time
import warnings

from robot_body import RobotBody
from hobot_vio import libsrcampy as camera_lib

import args
import utils
import global_constants as gc
from detector import YoloDetector
from robot_head import RobotHead


class AiAgent(object):
    def __init__(self, robot_body: RobotBody, robot_head: RobotHead, **kwargs):
        # general initialization
        self.robot_body = robot_body
        self.robot_head = robot_head
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'ai_agent_config.yaml',
            **kwargs,
        )
        camera_kwargs = parameters['camera_kwargs']
        self.verbose = parameters['verbose']
        self.agent_active = False

        # camera initialization
        self.camera_is_open = -1
        self.camera = camera_lib.Camera()
        self.video_capture_kwargs = camera_kwargs['video_capture_kwargs']
        self.frame_width = self.video_capture_kwargs['width']
        self.frame_height = self.video_capture_kwargs['height']

        # yolo detector initialization
        image_size = (self.frame_width, self.frame_height)
        self.detector = YoloDetector(robot_head=robot_head, image_size=image_size, verbose=self.verbose)
        self.save_images = parameters['save_images']

        # motion initialization
        self.steer_threshold = parameters['steer_threshold']
        self.angular_speed_range = parameters['angular_speed_range']
        self.speed_x = 0
        self.speed_z = 0

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
            self.robot_head.turn_off_light()
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

    # stop -> observe -> think -> move for 0.3 seconds -> repeat until interrupted
    def detect_and_move(self) -> None:
        if self.verbose >= 2:
            start_thinking = time.time()
        # show thinking light (blue)
        # self.robot_body.set_colorful_lamps(led_id=gc.ALL_LIGHTS_ID, red=0, green=0, blue=255)
        self.set_zero_speed()
        self.robot_body.set_car_motion(self.speed_x, 0, self.speed_z)
        move_duration = 0.3

        self.camera.get_img(2)
        self.camera.get_img(2)
        frame = self.camera.get_img(2)
        if frame is None:
            if self.verbose >= 1:
                print('Frame is None.')
            time.sleep(0.5)
            return
        frame = utils.format_camera_frames(
            frame=frame,
            width=self.frame_width,
            height=self.frame_height,
            # save_img=save_img,
        )

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
        # print(f'num_targets: {target_info["num_targets"]}')
        if target_info['num_targets'] > 0:
            # show target-found light (green)
            # self.robot_body.set_colorful_lamps(led_id=gc.ALL_LIGHTS_ID, red=0, green=255, blue=0)
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
                move_duration = 1.0
        else:
            # show target-not-found/searching light (orange)
            # self.robot_body.set_colorful_lamps(led_id=gc.ALL_LIGHTS_ID, red=255, green=127, blue=0)
            if self.verbose >= 2:
                print('Searching...')
            # TODO: very slowly rotate by 360° degree
            self.speed_x = 0
            self.speed_z = self.robot_head.speed_coefficient * 5
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
