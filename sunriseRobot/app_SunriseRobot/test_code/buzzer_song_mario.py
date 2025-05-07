from .. import args
from robot_body import RobotBody
from .. import global_constants as gc


kwargs = {}
parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'main_thread.yaml', **kwargs)

robot_body = RobotBody(com=parameters['com'], baud_rate=parameters['baud_rate'], verbose=parameters['verbose'])
robot_body.create_receive_threading()
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(300)
robot_body.set_beep(300)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(300)
robot_body.set_beep(300)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(300)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(100)
robot_body.set_beep(400)
