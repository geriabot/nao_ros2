from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
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
            output='screen'
        ),

        # NAO walk odometry node
        Node(
            package='nao_ros2',
            executable='nao_walk_odometry',
            name='nao_walk_odometry_node',
            output='screen'
        ),
    ])
