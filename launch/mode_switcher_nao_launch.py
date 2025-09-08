from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Base nodes
        Node(
            package='nao_ik',
            executable='nao_ik',
            name='nao_ik'
        ),
        Node(
            package='nao_phase_provider',
            executable='nao_phase_provider',
            name='nao_phase_provider',
            remappings=[('fsr', '/sensors/fsr')]
        ),
        Node(
            package='walk',
            executable='walk',
            name='walk'
        ),
        # Mode switcher logic node
        Node(
            package='nao_ros2',
            executable='mode_switcher_nao',
            name='mode_switcher_nao',
            output='screen',
            shell=True 
        ),
        # Include experiment_nao_launch.py
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         FindPackageShare('hni_cpp'), '/launch/experiment_nao_launch.py'
        #     ])
        # ),
    ])
