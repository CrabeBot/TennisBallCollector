import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node



def generate_launch_description():
    # Ball Tracking
    ball_tracking = Node(
        package="balls_localization",
        #condition=IfCondition(LaunchConfiguration("manager")),
        #parameters=[{"use_sim_time": True}],
        output="screen",
        #emulate_tty=True,
        executable="b_localizer"
    )    

    return LaunchDescription([
        ball_tracking,
    ])
