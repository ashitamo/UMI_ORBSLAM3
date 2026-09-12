#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_share = get_package_share_directory(
        'umi_orbslam3_bringup'
    )

    default_param_file = os.path.join(
        bringup_share,
        'config',
        'xsens_mapping_100hz.yaml'
    )

    param_file = LaunchConfiguration('param_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=default_param_file,
            description='Path to Xsens parameter YAML file'
        ),

        SetEnvironmentVariable(
            'RCUTILS_LOGGING_USE_STDOUT',
            '1'
        ),

        SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM',
            '1'
        ),

        Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='screen',
            parameters=[param_file]
        )
    ])