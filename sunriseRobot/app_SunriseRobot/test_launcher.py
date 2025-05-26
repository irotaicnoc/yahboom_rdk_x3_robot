# import args
# import utils
# import global_constants as gc
# from robot_body import RobotBody
from test_code import inverse_kinematic


def main(**kwargs):
    # parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'main_thread.yaml', **kwargs)
    # robot_body = RobotBody(com=parameters['com'], baud_rate=parameters['baud_rate'], verbose=parameters['verbose'])
    # robot_body.create_receive_threading()

    inverse_kinematic.main()


if __name__ == '__main__':
    main()
