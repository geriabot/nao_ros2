from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    joy_teleop_share_dir = get_package_share_directory('nao_ros2')
    teleop_config = os.path.join(joy_teleop_share_dir, 'config', 'teleop.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop_node',
            output='screen',
            parameters=[teleop_config],
        ),
    ])