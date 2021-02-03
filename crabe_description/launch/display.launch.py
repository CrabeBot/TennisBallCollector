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


# Sous ROS2, un launch file doit nécessairement contenir la fonction generate_launch_description() 
# et retourner un LaunchDescription object.

def generate_launch_description():

    pkg_share = FindPackageShare(package='crabe_description').find('crabe_description')
    pkg_share_gazebo_ros = FindPackageShare("gazebo_ros").find("gazebo_ros")
    default_model_path = os.path.join(pkg_share, 'urdf/crabebot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config/display.rviz')

    ld = LaunchDescription(
        [DeclareLaunchArgument(name='robot_desc', default_value=default_model_path, description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path, description='Absolute path to rviz config file')
        ])

    # A l'intérieur de cette fonction, on crée les noeuds, avec pour chaque noeud le minimum vital,
    # càd : le nom du package et le nom de l'exécutable.
    # Il est possible d'ajouter + de customisation aux nodes si besoin.
    gazebo_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share_gazebo_ros, '/launch/gazebo.launch.py']),
             )

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    gazebo_spawn_entity_node = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-entity', 'crabebot', '-topic', '/robot_description', '-y', '10'],
                        output='screen')


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable='robot_state_publisher',
        #parameters=[{'robot_description': LaunchConfiguration('robot_desc')}]
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('robot_desc')])}]
    )

    rqt_robot_steering_node = Node(
        package="rqt_robot_steering",
        executable='rqt_robot_steering',
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )


    # On ajoute tous les noeuds créés à la LaunchDescription
    # Sans cet ajout, les noeuds ne seront pas lancés.
    ld.add_action(gazebo_node)
    ld.add_action(gazebo_spawn_entity_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rqt_robot_steering_node)
    ld.add_action(rviz_node)

    # La fonction retourne le LaunchDescription object, qui contient tous les noeuds à lancer.
    return ld