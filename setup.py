from setuptools import find_packages, setup

package_name = 'nao_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/mode_switcher_nao_launch.py',
            'launch/mode_switcher_pc_launch.py',
            'launch/nao_description_launch.py',
            'launch/pointcloud2laserscan_launch.py',
            'launch/imu_ekf_launch.py',
            'launch/controller_joy_launch.py',
            'launch/nao_odometry_launch.py',
            'launch/nao.launch.py',
            'launch/joy_teleop.launch.py',
            'launch/leds.launch.py',
        ]),
        ('share/' + package_name + '/config', [
        'config/teleop.yaml',
        'config/nao_default.yaml',
    ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antonio Roldan',
    maintainer_email='andoniroldansandua@gmail.com',
    description='ROS2 package for handling and switching modes of the NAO robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_switcher_nao = nao_ros2.mode_switcher_nao:main',
            'camera_conversion_node = nao_ros2.camera_conversion_node:main',
            'nao_imu_odometry = nao_ros2.nao_imu_odometry:main',
            'imu_transformer = nao_ros2.imu_transformer:main',
            'imu_comparer = nao_ros2.imu_comparer:main',
            'nao_walk_odometry = nao_ros2.nao_walk_odometry:main',
            'set_leds_node = nao_ros2.set_leds_node:main',
            'test_leds_node = nao_ros2.test_leds_node:main',
        ],
    },
)
