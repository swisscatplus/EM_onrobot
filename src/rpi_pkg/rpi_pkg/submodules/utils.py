import yaml
import logging

# def configure_logger():
#     .logger = logging.getLogger(__name__)
#     .logger.setLevel(logging.DEBUG)
#     ch = logging.StreamHandler()

#     ch.setLevel(logging.INFO)
#     formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
#     ch.setFormatter(formatter)
#     .logger.addHandler(ch)

def get_config_yaml(config_file_path=None):
  with open(config_file_path, 'r') as file:
      data = yaml.safe_load(file)
  return data