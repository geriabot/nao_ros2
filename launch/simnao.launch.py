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
            name='walk',
            parameters=[{ # Walk parameters optimized for simulator
                'max_forward': 0.1,  # max forward velocity (m/s)
                'max_left': 0.05,  # max side velocity (m/s)
                'max_turn': 0.5,  # max turn velocity (rad/s)
                'speed_multiplier': 0.8,  # how much to multiply speed by (0.0 - 1.0)
                'foot_lift_amp': 0.015,  # how much to raise foot when it is highest (m) - increased for simulator
                'period': 0.35,  # time taken for one step (s) - slightly slower for simulator stability
                'dt': 0.01,  # time between each generateCommand call (s)
                'sole_x': 0.0,  # x coordinate of sole from hip when standing (m) - centered for better turning
                'sole_y': 0.053,  # y coordinate of sole from hip when standing (m)
                'sole_z': -0.310,  # z coordinate of sole from hip when standing (m)
                'max_forward_change': 0.04,  # how much forward can change in one step (m/s)
                'max_left_change': 0.04,  # how much left can change in one step (m/s)
                'max_turn_change': 0.6,  # how much turn can change in one step (rad/s)
                'footh_forward_multiplier': 0.2,  # extra height multiplier for forward steps - increased for simulator
                'footh_left_multiplier': 0.25,  # extra height multiplier for side steps - increased for simulator
                'arm_base_position_left': 1.7,
                'arm_base_position_right': 1.7,
                'arm_swing_amplitude': 0.1,
                'arm_step_size': 0.015,
                'arm_min_twist_to_activate': 0.05,
            }]
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
            ]),
            launch_arguments={
            'run_interaction_services': 'true',
            'start_sound_play': 'false',
            }.items()
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

        # Odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("nao_ros2"), "/launch/nao_odometry_launch.py"
            ])
        ),
    ])
