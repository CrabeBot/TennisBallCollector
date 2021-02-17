
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    path_planner = Node(package='path_planner', executable='path_planner',
                        output='screen')

    return LaunchDescription([
        path_planner,
    ])