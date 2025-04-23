import time

import args
import global_constants as gc
from robot_body import RobotBody


kwargs = {}
parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'main_thread.yaml', **kwargs)

robot_body = RobotBody(com=parameters['com'], baud_rate=parameters['baud_rate'], verbose=parameters['verbose'])
robot_body.create_receive_threading()
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
time.sleep(0.1)
robot_body.set_beep(600)
robot_body.set_beep(600)
robot_body.set_beep(600)
robot_body.set_beep(200)
time.sleep(0.2)
robot_body.set_beep(200)
robot_body.set_beep(800)
