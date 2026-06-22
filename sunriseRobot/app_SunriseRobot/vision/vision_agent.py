import time
import warnings

from hobot_vio import libsrcampy as camera_lib

import args
import utils
import global_constants as gc
from vision.detector import YoloDetector
from ros2.lidar_listener import ThreadedLidarListener


class VisionAgent(object):
    def __init__(self, robot_body, robot_head, **kwargs):
        # general initialization
        parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'vision_agent.yaml', **kwargs)
        self.robot_body = robot_body
        self.robot_head = robot_head
        self.verbose = parameters['verbose']
        # when True, print a per-frame timing breakdown (camera grab + detector stages) to diagnose the
        # ~3 s vision latency. Passed down to the detector so its inference/NMS split is printed too.
        self.profile = parameters['profile']
        self.agent_active = False
        self.think_steps_if_no_target = parameters['think_steps_if_no_target']
        self.no_target_counter = 0

        # camera initialization
        self.camera_is_open = -1
        self.camera = camera_lib.Camera()
        self.video_capture_kwargs = parameters['video_capture_kwargs']
        self.frame_width = self.video_capture_kwargs['width']
        self.frame_height = self.video_capture_kwargs['height']

        # yolo detector initialization
        self.detector = YoloDetector(
            camera_image_size=(self.frame_width, self.frame_height),
            verbose=self.verbose,
            profile=self.profile,
        )
        self.save_images = parameters['save_images']

        # motion initialization
        self.steer_threshold_1 = parameters['steer_threshold_1']
        self.steer_threshold_2 = parameters['steer_threshold_2']
        self.angular_speed_range = parameters['angular_speed_range']
        self.speed_x = 0
        self.speed_z = 0
        self.move_duration = parameters['move_duration']

        # lidar initialization
        self.lidar_listener = None
        self.lidar_is_active = False
        self.lidar_parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'lidar_listener.yaml',
            verbose=self.verbose,
        )
        self.target_distance = self.lidar_parameters['response_dist']
        self.target_reached_distance = parameters['target_reached_distance']

        # tri cable led
        self.led_3_pin = robot_head.led_3_pin
        self.use_led_3_pin = parameters['use_led_3_pin']

        # external headlight (RC LED light bar): turn it on automatically when the scene is dark
        headlight_parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'headlight.yaml',
            verbose=self.verbose,
        )
        self.headlight = robot_head.headlight
        self.auto_headlight = headlight_parameters['auto_in_vision_mode'] and self.headlight is not None
        self.auto_brightness_threshold = headlight_parameters['auto_brightness_threshold']
        self._auto_headlight_on = False

    def set_zero_speed(self):
        self.speed_x = 0
        self.speed_z = 0
        self.robot_body.set_car_motion(0, 0, 0)

    def deactivate_agent(self):
        if self.verbose >= 1:
            print('Deactivating vision agent...')
        self.set_zero_speed()
        self.agent_active = False
        if self.camera_is_open == 0:
            self.camera_is_open = -1
            self.camera.close_cam()
            if self.verbose >= 2:
                print('Camera closed.')

        # turn off tri cable led
        if self.use_led_3_pin:
            self.led_3_pin.set_color(gc.POWER_OFF)

        # hand the headlight back when leaving vision mode (turn off only what auto control turned on)
        if self._auto_headlight_on:
            self.headlight.turn_off()
            self._auto_headlight_on = False

        # destroy lidar listener
        if self.lidar_is_active:
            self.lidar_listener.delete_listener()
        self.lidar_is_active = False
        self.lidar_listener = None

    def activate_agent(self, video_capture_kwargs=None):
        if self.verbose >= 1:
            print('Activating vision agent...')
        self.set_zero_speed()
        self.camera_is_open = -1
        if video_capture_kwargs is None:
            self.camera_is_open = self.camera.open_cam(**self.video_capture_kwargs)
        else:
            self.camera_is_open = self.camera.open_cam(**video_capture_kwargs)

        if self.camera_is_open == 0:
            self.agent_active = True
            self.led_3_pin.set_color(gc.POWER_OFF)
            if self.verbose >= 2:
                print('Camera opened correctly.')
        else:
            warnings.warn('Failed to open camera.')
            warnings.warn('Impossible to run vision agent.')
            self.agent_active = False
            raise Exception('Failed to open camera.')

        # start lidar listener
        try:
            self.lidar_listener = ThreadedLidarListener(**self.lidar_parameters)
            self.target_distance = self.lidar_parameters['response_dist']
            self.lidar_is_active = True
        # if there is an error, run vision agent without lidar
        except Exception as e:
            utils.print_exception(exception=e, message='Failed to start lidar listener for Vision agent with error')
            self.lidar_is_active = False
            self.lidar_listener = None

    def autonomous_behavior(self):
        if self.robot_head.robot_mode == gc.MODE_AUTONOMOUS_VISION:
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
        if self.use_led_3_pin:
            self.led_3_pin.set_color(gc.RED)
        if self.verbose >= 3:
            start_thinking = time.time()
        self.set_zero_speed()

        if self.profile:
            grab_start = time.perf_counter()
        self.camera.get_img(2)
        self.camera.get_img(2)
        frame = self.camera.get_img(2)
        if self.profile:
            print(f'[profile] camera grab (3x get_img)={(time.perf_counter() - grab_start) * 1000:.0f} ms')
        if frame is None:
            if self.verbose >= 1:
                print('Frame is None.')
            time.sleep(0.5)
            return

        self._update_auto_headlight(frame)

        target_info = self.detector.find_target(
            frame=frame,
            model_name=self.robot_head.vision_model_list[self.robot_head.vision_model_pos],
            target_name=self.robot_head.tracking_target_list[self.robot_head.tracking_target_pos],
            save=self.save_images,
        )
        # target_info = {
        #     'num_targets': int,
        #     'highest_confidence': float [0, 1],
        #     'distance_from_center_x': float [-1, 1],
        #     'distance_from_center_y': float [-1, 1],
        # }
        if self.verbose >= 3:
            print(f'num_targets: {target_info["num_targets"]}')
        if target_info['num_targets'] > 0:
            # show target-found light (green)
            if self.use_led_3_pin:
                self.led_3_pin.set_color(gc.GREEN)
            self.no_target_counter = 0
            distance_from_center_x = target_info['distance_from_center_x']
            if self.verbose >= 3:
                print(f'X distance from img center: {distance_from_center_x}')

            # if the robot is almost aligned with the target (angle < steer_threshold_1)
            #     the robot will advance
            # if the robot is somewhat aligned with the target (steer_threshold_1 < angle < steer_threshold_2)
            #     the robot will steer AND advance
            # if the robot is not aligned with the target (angle > steer_threshold_2)
            #     the robot will steer
            # otherwise move forward
            if abs(distance_from_center_x) > self.steer_threshold_2:
                self.speed_x = 0
                self.speed_z = utils.x_displacement_to_angular_speed(
                    x_distance_from_img_center=distance_from_center_x,
                    steer_threshold=self.steer_threshold_2,
                    angular_speed_range=self.angular_speed_range,
                )

            elif self.steer_threshold_1 < abs(distance_from_center_x) < self.steer_threshold_2:
                self.speed_x = self.robot_head.speed_coefficient / 2
                self.speed_z = utils.x_displacement_to_angular_speed(
                    x_distance_from_img_center=distance_from_center_x,
                    steer_threshold=self.steer_threshold_1,
                    angular_speed_range=self.angular_speed_range,
                )

            else:
                self.speed_x = self.robot_head.speed_coefficient
                self.speed_z = 0

                if self.lidar_is_active:
                    self.target_distance = self.lidar_parameters['response_dist']
                    obstacles_by_sector, average_distance_by_sector = self.lidar_listener.get_obstacles_by_sector()
                    if obstacles_by_sector is not None:
                        # check if there are obstacles in the front
                        if obstacles_by_sector[0]:
                            self.target_distance = average_distance_by_sector[0]
                        if obstacles_by_sector[-1]:
                            self.target_distance = min(average_distance_by_sector[-1], self.target_distance)
                        if self.target_distance <= self.target_reached_distance:
                            # stop the robot
                            self.speed_x = 0
                            # target reached!
                            if self.verbose >= 1:
                                print('Target reached!')
                            if self.use_led_3_pin:
                                self.led_3_pin.set_color(gc.GREEN)
                            self.robot_body.set_beep(gc.LONG_BEEP)
                    if self.target_distance == self.lidar_parameters['response_dist']:
                        print(f'Target farther than {self.target_distance} m')
                    else:
                        print(f'Target distance: {int(self.target_distance * 100)} cm')

            if self.verbose >= 2:
                print(f'Forward: {self.speed_x}')
                print(f'Steer: {self.speed_z}')

        else:
            # show target-not-found/searching light (orange)
            if self.use_led_3_pin:
                self.led_3_pin.set_color(gc.ORANGE)

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
                if self.verbose >= 3:
                    print(f'Forward: {self.speed_x}')
                    print(f'Steer: {self.speed_z}')
        if self.verbose >= 3:
            stop_thinking = time.time()
            print(f'thinking time: {round(stop_thinking - start_thinking, 3)}')

        # start_moving = time.time()
        self.robot_body.set_car_motion(self.speed_x, 0, self.speed_z)
        time.sleep(self.move_duration)
        # stop_moving = time.time()
        # print(f'moving time AI: {round(stop_moving - start_moving, 3)}')

    def _update_auto_headlight(self, frame) -> None:
        # turn the external headlight on automatically when the scene is dark. Latched: once on it
        # stays on until vision mode ends (deactivate_agent), to avoid it flickering off as soon as
        # its own light brightens the view.
        if not self.auto_headlight or self._auto_headlight_on:
            return
        brightness = utils.y_plane_mean_brightness(frame, self.frame_width, self.frame_height)
        if brightness < self.auto_brightness_threshold:
            self.headlight.turn_on()
            self._auto_headlight_on = True
            if self.verbose >= 1:
                print(f'Dark scene (brightness {brightness:.0f} < {self.auto_brightness_threshold}); '
                      f'turning headlight on.')

    def __del__(self):
        self.deactivate_agent()
