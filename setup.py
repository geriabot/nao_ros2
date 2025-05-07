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
        ],
    },
)
