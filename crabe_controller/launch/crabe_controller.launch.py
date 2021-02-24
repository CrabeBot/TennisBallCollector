
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    controller = Node(package='crabe_controller', executable='crabe_controller',
                        output='screen')

    return LaunchDescription([
        controller,
    ])