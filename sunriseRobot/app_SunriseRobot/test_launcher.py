import args
# import utils
import global_constants as gc
from robot_body import RobotBody
from test_code import buzzer_song_ff
from test_code import buzzer_song_mario


def main(**kwargs):
    parameters = args.import_args(yaml_path=gc.CONFIG_FOLDER_PATH + 'main_thread.yaml', **kwargs)
    robot_body = RobotBody(com=parameters['com'], baud_rate=parameters['baud_rate'], verbose=parameters['verbose'])
    robot_body.create_receive_threading()

    buzzer_song_ff.play(robot_body=robot_body)
    # buzzer_song_mario.play(robot_body=robot_body)


if __name__ == '__main__':
    main()