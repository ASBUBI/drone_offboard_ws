# launch file for vicon_stream node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    params_file_path = os.path.join(get_package_share_directory(
        'vicon_stream'), 'config', 'vicon_stream_params.yaml')
    
    logging_level = 'INFO'

    vicon_client = Node(
        package = 'vicon_stream',
        executable = 'vicon_client',
        name = 'vicon_client',
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
        vicon_client
    ])