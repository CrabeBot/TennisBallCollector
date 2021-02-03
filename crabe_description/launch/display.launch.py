import os 
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("crabe_description").find("crabe_description")
    model_file = os.path.join(pkg_share, "urdf", "crabebot.urdf")
    rviz_config_file = os.path.join(pkg_share, "config", "config.rviz")

    robot_state_publisher_node = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro", " ", model_file])}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])