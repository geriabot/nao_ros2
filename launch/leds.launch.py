from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('nao_ros2')

    led_config_file = os.path.join(pkg_share, 'config', 'nao_default.yaml')

    return LaunchDescription([
        Node(
            package='nao_ros2',
            executable='set_leds_node',
            name='set_leds_client',
            output='screen',
            parameters=[{'led_config_file': led_config_file}]
        )
    ])
