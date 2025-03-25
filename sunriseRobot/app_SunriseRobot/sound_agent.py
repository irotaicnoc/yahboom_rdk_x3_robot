import time
import warnings

import tuning
from robot_body import RobotBody

import args
import utils
import global_constants as gc
from robot_head import RobotHead
from gpio_pin_control import GpioLed


class SoundAgent(object):
    def __init__(self, robot_body: RobotBody, robot_head: RobotHead, gpio_led: GpioLed, **kwargs):
        # general initialization
        self.robot_body = robot_body
        self.robot_head = robot_head
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'sound_agent.yaml',
            **kwargs,
        )
        self.verbose = parameters['verbose']

        self.agent_active = False
        self.microphone = None

        # motion initialization
        self.steer_threshold_1 = parameters['steer_threshold_1']
        self.steer_threshold_2 = parameters['steer_threshold_2']
        self.angular_speed_range = parameters['angular_speed_range']
        self.vendor_id = parameters['vendor_id']
        self.product_id = parameters['product_id']
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
            print('Deactivating sound agent...')
        self.set_zero_speed()
        self.robot_body.set_car_motion(v_x=self.speed_x, v_y=0, v_z=self.speed_z)
        self.agent_active = False

        self.microphone.close()

        # turn off gpio led
        if self.use_gpio_led:
            self.gpio_led.set_color('off')

    def activate_agent(self, video_capture_kwargs=None):
        if self.verbose >= 1:
            print('Activating sound agent...')
        self.set_zero_speed()
        self.robot_body.set_car_motion(self.speed_x, 0, self.speed_z)

        # microphone initialization
        # ReSpeaker 4-Mic Array v2.0
        self.microphone = tuning.find(vid=self.vendor_id, pid=self.product_id)

        if self.microphone:
            self.agent_active = True
            self.gpio_led.set_color('off')
            if self.verbose >= 2:
                print('Microphone opened correctly.')
        else:
            warnings.warn('Failed to open microphone.')
            warnings.warn('Impossible to run sound agent.')
            self.agent_active = False

    def autonomous_behavior(self):
        if self.robot_head.robot_mode == 'autonomous_sound':
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

    # stop -> listen -> think -> move for n seconds -> repeat until interrupted
    def detect_and_move(self) -> None:
        # show thinking light (red)
        if self.use_gpio_led:
            self.gpio_led.set_color('red')
        if self.verbose >= 2:
            start_thinking = time.time()
        self.set_zero_speed()
        self.robot_body.set_car_motion(self.speed_x, 0, self.speed_z)
        move_duration = 0.5

        # get the strongest sound direction as an angle
        # TODO: check this. 0° is the front of the robot, 90° is the right side of the robot, 180° is the back of the robot,
        #  270° is the left side of the robot
        target_angle = self.microphone.direction

        if self.verbose >= 2:
            print(f'target angle: {target_angle}')
        if target_angle:
            # show target-found light (green)
            if self.use_gpio_led:
                self.gpio_led.set_color('green')
            distance_from_center_x = target_info['distance_from_center_x']
            # print(f'target x: {distance_from_center_x}')
            # if the robot is almost aligned with the target (angle < steer_threshold_1)
            #     the robot will advance
            # if the robot is almost somewhat aligned with the target (steer_threshold_1 < angle < steer_threshold_2)
            #     the robot will steer AND advance
            # if the robot is not aligned with the target (angle > steer_threshold_2)
            #     the robot will steer
            # otherwise move forward
            if self.verbose >= 2:
                print(f'X distance from img center: {distance_from_center_x}')
            if abs(distance_from_center_x) > self.steer_threshold_2:
                self.speed_x = 0
                self.speed_z = utils.x_displacement_to_angular_speed(
                    x_distance_from_img_center=distance_from_center_x,
                    steer_threshold=self.steer_threshold_2,
                    angular_speed_range=self.angular_speed_range,
                )
                if self.verbose >= 2:
                    print(f'Steer: {self.speed_z}')

            elif self.steer_threshold_1 < abs(distance_from_center_x) < self.steer_threshold_2:
                self.speed_x = self.robot_head.speed_coefficient / 2
                self.speed_z = utils.x_displacement_to_angular_speed(
                    x_distance_from_img_center=distance_from_center_x,
                    steer_threshold=self.steer_threshold_2,
                    angular_speed_range=self.angular_speed_range,
                ) / 2
                if self.verbose >= 2:
                    print(f'Forward: {self.speed_x}')
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
