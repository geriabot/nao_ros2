from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'min_height': -0.5,
                'max_height': 1.0,
                'angle_min': -1.57,
                'angle_max': 1.57,
                'angle_increment': 0.0058,
                'range_min': 0.0,
                'range_max': 2.0,
                'scan_time': 0.01,
                'target_frame': 'camera_depth_frame',
                'use_inf': True 
            }],
            remappings=[('/cloud_in', '/astra/depth/points')]
        )
    ])
