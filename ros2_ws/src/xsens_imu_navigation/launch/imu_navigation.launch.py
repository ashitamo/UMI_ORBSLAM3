from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory('xsens_imu_navigation')
    parameters = os.path.join(package_share, 'config', 'imu_navigation.yaml')

    return LaunchDescription([
        Node(
            package='xsens_imu_navigation',
            executable='imu_dead_reckoning',
            name='imu_dead_reckoning',
            output='screen',
            parameters=[parameters],
        )
    ])
