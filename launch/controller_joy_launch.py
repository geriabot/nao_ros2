from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch blueman-manager for Bluetooth device management (GUI)
        ExecuteProcess(
            cmd=['blueman-manager'],
            output='screen'
        ),
        # Launch the joystick driver node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'device': '/dev/input/js0'}],
            output='screen'
        ),
        # Launch the teleop twist joy node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                {'enable_button': 4},
                {'axis_linear.x': 1},
                {'scale_linear.x': 0.5},
                {'axis_angular.yaw': 0},
                {'scale_angular.yaw': 0.5}
            ],
            remappings=[
                ('/cmd_vel', '/target')
            ],
            output='screen'
        )
    ])
