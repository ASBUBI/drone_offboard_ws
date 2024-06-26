# launch file for logger node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    params_file_path = os.path.join(get_package_share_directory(
        'logger'), 'config', 'logger_params.yaml')
    
    logging_level = 'INFO'

    logger = Node(
        package = 'logger',
        executable = 'logger',
        name = 'logger',
        output = 'screen',
        emulate_tty = True,
        arguments = [
            '--ros-args',
            '--params-file',
            params_file_path,
            '--log-level',
            logging_level
        ]
    )

    return LaunchDescription([
        logger
    ])