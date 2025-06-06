from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_safety_controller',
            executable='behavior_tree',
            name='safety_controller',
            parameters=[os.path.join(get_package_share_directory('robot_safety_controller'), 'config', 'params.yaml')],
        )
    ])