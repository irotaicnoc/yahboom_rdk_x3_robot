import yaml
import copy
import argparse


def read() -> dict:
    with open('/root/marco_ros2_ws/src/controller_sub/controller_sub/config.yaml') as config_file:
        default_arg_dict = yaml.safe_load(config_file)

    parser = argparse.ArgumentParser()
    for key in default_arg_dict:

        value = default_arg_dict[key]
        parser.add_argument(f'--{key}', dest=key, type=type(value))

    updated_arg_dict = copy.deepcopy(default_arg_dict)
    args = parser.parse_args()
    arg_dict = vars(args)
    for key in arg_dict:
        if arg_dict[key] is not None:
            updated_arg_dict[key] = arg_dict[key]

    return updated_arg_dict
