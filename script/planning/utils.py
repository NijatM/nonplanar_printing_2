import os


def get_root_folder_path():
    current_file_location = os.path.abspath(__file__)
    current_folder_path = os.path.dirname(current_file_location)
    root_folder_path = os.path.dirname(os.path.dirname(current_folder_path))
    return root_folder_path


def remove_robot_model_visuals(robot_model):
    for link in robot_model.links:
        link.visual = []
