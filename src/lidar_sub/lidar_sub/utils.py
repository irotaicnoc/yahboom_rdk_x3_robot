# standard libraries
import yaml


def args_from_yaml(config_path: str) -> dict:
    with open(config_path) as config_file:
        arg_dict = yaml.safe_load(config_file)
    return arg_dict
