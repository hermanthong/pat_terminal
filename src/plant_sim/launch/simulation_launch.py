from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='plant_sim', executable='plant_sim_node', name='plant_sim'),
        Node(package='pat_terminal', executable='mode_manager', name='mode_manager'),
        Node(package='pat_terminal', executable='fine_controller', name='fine_controller'),
        Node(package='pat_terminal', executable='coarse_controller', name='coarse_controller'),
    ])
