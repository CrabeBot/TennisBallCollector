from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    player_localization = Node(package='player_localization', executable='p_localization',
                        output='screen')

    return LaunchDescription([
        player_localization,
    ])