import time

from SunriseRobotLib import SunriseRobot
from hobot_vio import libsrcampy as camera_lib

import args
import utils
from tracker import YoloTracker
from robot_head import RobotHead


class AiAgent(object):
    def __init__(self, robot_body: SunriseRobot, robot_head: RobotHead, **kwargs):
        # general initialization
        self.robot_body = robot_body
        self.robot_head = robot_head
        parameters = args.import_args(
            yaml_path='/root/sunriseRobot/app_SunriseRobot/configs/ai_agent_config.yaml',
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

        # yolo tracker initialization
        image_size = (self.frame_width, self.frame_height)
        self.tracker = YoloTracker(robot_head=robot_head, image_size=image_size, verbose=self.verbose)
        self.save_images = parameters['save_images']

        # motion initialization
        self.steer_threshold = parameters['steer_threshold']
        self.angular_speed_range = parameters['angular_speed_range']
        self.speed_x = 0
        self.speed_y = 0
        self.speed_z = 0

    def set_zero_speed(self):
        self.speed_x = 0
        self.speed_y = 0
        self.speed_z = 0

    def deactivate_agent(self):
        self.set_zero_speed()
        self.robot_body.set_car_motion(self.speed_x, self.speed_y, self.speed_z)
        self.agent_active = False
        if self.camera_is_open == 0:
            self.camera_is_open = -1
            self.camera.close_cam()
            if self.verbose:
                print('Camera closed.')

    def activate_agent(self, video_capture_kwargs=None):
        self.set_zero_speed()
        self.robot_body.set_car_motion(self.speed_x, self.speed_y, self.speed_z)
        self.camera_is_open = -1
        if video_capture_kwargs is None:
            self.camera_is_open = self.camera.open_cam(**self.video_capture_kwargs)
        else:
            self.camera_is_open = self.camera.open_cam(**video_capture_kwargs)

        if self.camera_is_open == 0:
            self.agent_active = True
            if self.verbose:
                print('Camera opened correctly.')
        else:
            print('Failed to open camera.')
            print('Impossible to run autonomous behavior.')
            self.agent_active = False

    def autonomous_behavior(self):
        if self.robot_head.robot_mode == 'autonomous_tracking':
            if self.agent_active:
                self.detect_and_move()
            else:
                if self.verbose:
                    print('Start autonomous behavior.')
                self.activate_agent()
                self.detect_and_move()
        else:
            if self.agent_active:
                if self.verbose:
                    print('Stop autonomous behavior.')
                self.deactivate_agent()
                time.sleep(2)
            else:
                time.sleep(2)

    # stop -> observe -> think -> move for 0.3 seconds -> repeat until interrupted
    def detect_and_move(self) -> None:
        self.set_zero_speed()
        self.robot_body.set_car_motion(self.speed_x, self.speed_y, self.speed_z)

        _ = self.camera.get_img(2)
        _ = self.camera.get_img(2)
        frame = self.camera.get_img(2)
        if frame is None:
            if self.verbose:
                print('Frame is None.')
            time.sleep(0.5)
            return
        frame = utils.format_camera_frames(
            frame=frame,
            width=self.frame_width,
            height=self.frame_height,
            # save_img=save_img,
        )

        target_info = self.tracker.find_target(
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
            distance_from_center_x = target_info['distance_from_center_x']
            # print(f'target x: {distance_from_center_x}')
            # only steer to the target if its center is more than
            # self.steer_threshold distant from the current forward direction
            # otherwise move forward
            if abs(distance_from_center_x) > self.steer_threshold:
                self.speed_x = 0
                self.speed_z = utils.x_displacement_to_angular_speed(
                    x_distance_from_img_center=distance_from_center_x,
                    steer_threshold=self.steer_threshold,
                    angular_speed_range=self.angular_speed_range,
                )

                print(f'Steer: {self.speed_z}')
            else:
                self.speed_x = self.robot_head.speed_coefficient
                print(f'Forward: {self.speed_x}')
                self.speed_z = 0
        else:
            print('Searching...')
            # TODO: slowly rotate by 360° degree
            self.set_zero_speed()
        self.robot_body.set_car_motion(self.speed_x, self.speed_y, self.speed_z)
        time.sleep(0.3)

    def __del__(self):
        self.deactivate_agent()
