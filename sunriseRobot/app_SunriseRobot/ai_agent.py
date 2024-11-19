import time

from SunriseRobotLib import SunriseRobot
from hobot_vio import libsrcampy as srcampy

import args
import utils
from tracker import YoloTracker


class AiAgent(object):
    def __init__(self, robot: SunriseRobot, **kwargs):
        # general initialization
        self.robot = robot
        parameters = args.import_args(yaml_path='/root/sunriseRobot/app_SunriseRobot/config.yaml', **kwargs)
        robot_kwargs = parameters['robot_kwargs']
        camera_kwargs = parameters['camera_kwargs']
        tracker_kwargs = parameters['tracker_kwargs']
        self.verbose = parameters['verbose']
        self.agent_active = True

        # camera initialization
        self.camera = srcampy.Camera()
        self.video_capture_kwargs = camera_kwargs['video_capture_kwargs']
        self.activate_agent(video_capture_kwargs=self.video_capture_kwargs)
        self.frame_width = self.video_capture_kwargs['width']
        self.frame_height = self.video_capture_kwargs['height']
        self.image_orientation = camera_kwargs['image_orientation']

        # yolo tracker initialization
        image_size = (self.frame_width, self.frame_height)
        self.tracker = YoloTracker(**tracker_kwargs, image_size=image_size, verbose=self.verbose)
        self.save_images = parameters['save_images']

        # motion initialization
        self.steer_threshold: float = 0.05
        self.steering_speed_coefficient: float = robot_kwargs['speed_coefficient']
        self.forward_speed_coefficient: float = robot_kwargs['speed_coefficient'] / 5
        self.speed_x = 0
        self.speed_y = 0
        self.speed_z = 0


    def deactivate_agent(self):
        if self.camera_is_open == 0:
            self.camera_is_open = -1
            self.camera.close_cam()
            self.agent_active = False
            if self.verbose:
                print('camera closed')


    def activate_agent(self, video_capture_kwargs = None):
        self.camera_is_open = -1
        if video_capture_kwargs is None:
            self.camera_is_open = self.camera.open_cam(**self.video_capture_kwargs)
        else:
            self.camera_is_open = self.camera.open_cam(**video_capture_kwargs)
            
        if self.camera_is_open == 0:
            self.agent_active = True
            if self.verbose:
                print('camera opened correctly')
        else:
            print('failed to open camera')
            print('impossible to run autonomous behavior')
            self.agent_active = False


    def update_params(self, params: dict) -> None:
        for param_name, param_value in params.items():
            if param_name == 'target_name':
                self.tracker.select_target(param_value)

            elif hasattr(self, param_name):
                setattr(self, param_name, param_value)


    def autonomous_behavior(self) -> None:
        while self.agent_active:
            frame = self.camera.get_img(2)
            if frame is None:
                if self.verbose:
                    print('frame is None')
                time.sleep(0.5)
                continue
            frame = utils.format_camera_frames(
                frame=frame,
                width=self.frame_width,
                height=self.frame_height,
                image_orientation=self.image_orientation,
                # save_img=save_img,
            )

            target_info = self.tracker.find_target(frame=frame, save=self.save_images)
            # target_info = {
            #     'num_targets': int,
            #     'highest_confidence': float [0, 1],
            #     'normalized_center_x': float [-0.5, 0.5],
            #     'normalized_center_y': float [-0.5, 0.5],
            # }
            if target_info['num_targets'] > 0:
                normalized_center_x = target_info['normalized_center_x']
                # only steer to the target if its center is more than self.steer_threshold distant from the current forward direction
                if abs(normalized_center_x) > self.steer_threshold:
                    self.speed_x = 0
                    self.speed_z = normalized_center_x * self.steering_speed_coefficient
                else:
                    self.speed_x = self.forward_speed_coefficient
                    self.speed_z = 0
                self.robot.set_car_motion(self.speed_x, self.speed_y, self.speed_z)

            time.sleep(0.5)
