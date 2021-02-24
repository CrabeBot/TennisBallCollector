import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    fsm = Node(package='state_machine', executable='fsm', output='screen')
    

    #### path planner
    pkg_share_path_planner = FindPackageShare(package='path_planner').find('path_planner')
    path_planner = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share_path_planner, '/launch/path_planner.launch.py']),
             )


    #### Robot
    pkg_share_crabe = FindPackageShare(package='crabe_description').find('crabe_description')
    crabe = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share_crabe, '/launch/display.launch.py']),
             )


    #### localisation des balles
    pkg_share_ball_loc = FindPackageShare(package='balls_localization').find('balls_localization')
    ball_localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share_ball_loc, '/launch/ball_tracking.launch.py']),
             )


    #### Controleur
    pkg_share_controller = FindPackageShare(package='crabe_controller').find('crabe_controller')
    crabe_controller = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share_controller, '/launch/crabe_controller.launch.py']),
             )


    #### localisation du robot
    pkg_share_crabe_loc = FindPackageShare(package='crabe_localization').find('crabe_localization')
    crabe_localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share_crabe_loc, '/launch/localization.launch.py']),
             )


    #### localisation des joueurs
    pkg_share_player_loc = FindPackageShare(package='player_localization').find('player_localization')
    player_localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share_player_loc, '/launch/player_localization.launch.py']),
             )
    



    return LaunchDescription([
        fsm,
        path_planner,
        crabe,
        ball_localization,
        crabe_controller,
        crabe_localization,
        player_localization
    ])