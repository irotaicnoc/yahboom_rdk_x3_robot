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
        self.no_sound_counter = 0

        # microphone initialization
        self.microphone = None
        self.microphone_robot_angle = parameters['microphone_robot_angle']
        self.ignore_self_noise = parameters['ignore_self_noise']
        self.ignore_angle = parameters['ignore_angle']

        # motion initialization
        self.angular_speed_range = parameters['angular_speed_range']
        self.forward_speed_range = parameters['forward_speed_range']
        self.turn_only_angle = parameters['turn_only_angle']
        self.product_id = parameters['product_id']
        self.vendor_id = parameters['vendor_id']
        self.speed_x = 0
        self.speed_z = 0
        self.move_duration = parameters['move_duration']

        # gpio led
        self.gpio_led = gpio_led
        self.use_gpio_led = parameters['use_gpio_led']

    def set_zero_speed(self):
        self.speed_x = 0
        self.speed_z = 0
        self.robot_body.set_car_motion(v_x=0, v_y=0, v_z=0)

    def deactivate_agent(self):
        if self.verbose >= 1:
            print('Deactivating sound agent...')
        self.set_zero_speed()
        self.agent_active = False
        self.microphone.close()

        # turn off gpio led
        if self.use_gpio_led:
            self.gpio_led.set_color('off')

    def activate_agent(self):
        if self.verbose >= 1:
            print('Activating sound agent...')
        self.set_zero_speed()

        # microphone initialization
        # ReSpeaker 4-Mic Array v2.0
        self.microphone = tuning.find(vid=self.vendor_id, pid=self.product_id)

        if self.microphone:
            self.agent_active = True
            self.gpio_led.set_color('off')
            if self.verbose >= 1:
                print('Microphone opened correctly.')
        else:
            warnings.warn('Failed to open microphone.')
            warnings.warn('Impossible to run sound agent.')
            self.agent_active = False
            raise Exception('Failed to open microphone.')

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

    def no_sound_detected(self):
        # show target-not-found/searching light (red_and_green)
        if self.use_gpio_led:
            self.gpio_led.set_color('red_and_green')

        self.speed_x = 0
        self.speed_z = 0
        self.no_sound_counter += 1
        # if self.verbose >= 2:
        #     if self.no_sound_counter % 20 == 19:
        #         print('No sound detected')

        time.sleep(0.05)

    # stop -> listen -> think -> move for n seconds -> repeat until interrupted
    def detect_and_move(self) -> None:
        # show thinking light (red)
        if self.use_gpio_led:
            self.gpio_led.set_color('red')
        if self.verbose >= 2:
            start_thinking = time.time()
        self.set_zero_speed()

        # get the strongest sound direction as an angle
        # anti-clockwise from 0 to 360 degrees
        target_angle_microphone = self.microphone.direction
        # if self.verbose >= 2:
        #     print(f'target_angle_microphone: {target_angle_microphone}')
        if target_angle_microphone:
            # show target-found light (green)
            if self.use_gpio_led:
                self.gpio_led.set_color('green')

            # convert the sound direction angle from the microphone to the robot
            # 0 is in front of the robot
            # 90 is on the left side of the robot
            # -90 is on the right side of the robot
            # 179/-180 is behind the robot
            target_angle_robot = utils.microphone_angle_to_robot_angle(
                direction_of_arrival=target_angle_microphone,
                microphone_robot_angle=self.microphone_robot_angle,
            )
            # if self.verbose >= 2:
            #     print(f'target_angle_robot: {target_angle_robot}')

            # if ignore_self_noise is True, the robot will ignore all sounds that come from the back of the
            # microphone array this will remove the sounds of the robot, but also the sounds of the target if
            # it is behind the robot
            if self.ignore_self_noise:
                if abs(target_angle_robot) > self.ignore_angle:
                    self.no_sound_detected()
                    return

            self.no_sound_counter = 0
            # if the target is behind or almost behind the robot (angle => turn_only_angle),
            #   the robot will only rotate in place
            # if the robot is almost somewhat aligned with the target (0 <= angle < turn_only_angle), the
            #   robot will steer AND advance. The proportion between steering and advancing is determined by the angle

            if abs(target_angle_robot) > self.turn_only_angle:
                self.speed_x = 0
                self.speed_z = self.angular_speed_range[1] * self.robot_head.speed_coefficient
                if target_angle_robot < 0:
                    self.speed_z *= -1

            else:
                self.speed_x, self.speed_z = utils.sound_angle_to_robot_speed(
                    sound_angle=target_angle_robot,
                    turn_only_angle=self.turn_only_angle,
                    forward_speed_range=self.forward_speed_range,
                    angular_speed_range=self.angular_speed_range,
                )
                self.speed_x *= self.robot_head.speed_coefficient
                self.speed_z *= self.robot_head.speed_coefficient

            # if self.verbose >= 2:
            #     print(f'thinking time: {round(time.time() - start_thinking, 3)}')
            #     print(f'Forward: {self.speed_x}')
            #     print(f'Steer: {self.speed_z}')
            self.robot_body.set_car_motion(self.speed_x, 0, self.speed_z)
            time.sleep(self.move_duration)

        else:
            self.no_sound_detected()

    def __del__(self):
        self.deactivate_agent()
