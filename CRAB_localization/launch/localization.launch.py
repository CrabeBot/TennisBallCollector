from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    loc_node = Node(
        package='CRAB_localization',
        executable='localizer',
    )

    ld.add_action(loc_node)

    # La fonction retourne le LaunchDescription object, qui contient tous les noeuds à lancer.
    return ld