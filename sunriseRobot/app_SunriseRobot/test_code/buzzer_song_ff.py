import time

from robot_body import RobotBody


robot_body = RobotBody(verbose=2)
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
