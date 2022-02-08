from easydict import EasyDict
import yaml


def load_config(config_file: str):
    return EasyDict(yaml.safe_load(open(config_file)))


def write_config(config: EasyDict, filename: str):
    with open(filename, "w") as outfile:
        yaml.dump(config, outfile, default_flow_style=False)
