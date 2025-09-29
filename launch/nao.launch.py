from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nao_lola_client',
            executable='nao_lola_client',
            name='lola_node',
            output='screen',
        ),
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
        # Interaction
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('simple_hri'), '/launch/simple_hri.launch.py'
                ])
            ),
        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('nao_ros2'), '/launch/nao_description_launch.py'
            ])
        ),
        # Camera conversion node
        Node(
            package='nao_ros2',
            executable='camera_conversion_node',
            name='camera_conversion_node',
            output='screen',
            shell=True 
        ),
        # Swing legs
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("nao_pos_server"), '/launch', '/swing_launch.py'])
        ),

        # Posture server
        Node(
            package='nao_pos_server',
            executable='nao_pos_action_server',
            name='nao_pos_action_server',
            output='screen'
        ),

        # LEDs server
        Node(
            package='nao_led_server',
            executable='led_action_server',
            name='led_action_server_node',
            output='screen'
        ),

        # LEDs initialization
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("nao_ros2"), '/launch', '/leds.launch.py'])
        ),

        # Sound play
        Node(
            package='sound_play',
            executable='soundplay_node.py',
            name='soundplay_node',
            output='screen',
        ),

        # Image
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
        ),

        # Sonar activation
        Node(
            package='nao_ros2',
            executable='activate_sonar',
            name='activate_sonar_node',
        ),
    ])
