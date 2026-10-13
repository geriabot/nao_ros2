from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Whether the IMU yaw has the opposite sign to the robot's (true on the robot these
        # nodes were written for; false in the Webots simulation): parameters invert_imu_yaw
        # of nao_walk_odometry and flip_ned_to_enu of imu_transformer (read by the EKF)
        DeclareLaunchArgument('invert_imu_yaw', default_value='true'),

        # Include NAO robot description launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("nao_ros2"), "/launch/nao_description_launch.py"
            ])
        ),

        # Include IMU EKF launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("nao_ros2"), "/launch/imu_ekf_launch.py"
            ])
        ),

        # Madgwick filter node
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            remappings=[
                ('/imu/data_raw', '/imu')
            ],
            parameters=[
                {'use_mag': False},
                {'remove_gravity_vector': True},
                {'publish_tf': False},
                {'world_frame': 'enu'}
            ]
        ),

        # IMU transformer node
        Node(
            package='nao_ros2',
            executable='imu_transformer',
            name='imu_transformer_node',
            output='screen',
            parameters=[{'flip_ned_to_enu': ParameterValue(
                LaunchConfiguration('invert_imu_yaw'), value_type=bool)}]
        ),

        # NAO walk odometry node
        Node(
            package='nao_ros2',
            executable='nao_walk_odometry',
            name='nao_walk_odometry_node',
            output='screen',
            parameters=[{'invert_imu_yaw': ParameterValue(
                LaunchConfiguration('invert_imu_yaw'), value_type=bool)}]
        ),
    ])
