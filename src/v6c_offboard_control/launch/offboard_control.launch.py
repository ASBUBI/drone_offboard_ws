# launch file for v6c_offboard_control 
# concatenate launch file for "vicon_stream" node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    params_file_path = os.path.join(get_package_share_directory(
      'v6c_offboard_control'), 'config', 'offboard_control_params.yaml')
    
    logging_level = 'INFO'

    offboard_control = Node(
        package = 'v6c_offboard_control',
        executable = 'offboard_control',
        name = 'offboard_control',
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

    vicon_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('vicon_stream'), 'launch'),
            '/vicon_stream.launch.py'])
    )

    return LaunchDescription([
        offboard_control,
        vicon_client
    ])
