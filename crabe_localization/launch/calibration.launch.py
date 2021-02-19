from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    loc_node = Node(
        package='crabe_localization',
        executable='localizer',
        parameters=[{'use_sim_time': True}]
    )

    cal_node = Node(
        package='crabe_localization',
        executable='calibration',
        parameters=[{'use_sim_time': True}]
    )

    ld.add_action(loc_node)
    ld.add_action(cal_node)

    # La fonction retourne le LaunchDescription object, qui contient tous les noeuds à lancer.
    return ld