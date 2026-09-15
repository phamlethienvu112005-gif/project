import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    static_tf_node = Node(
        package='amr_hardware',
        executable='static_tf_broadcaster',
        name='static_tf_broadcaster'
    )

    esp32_node = Node(
        package='amr_hardware',
        executable='esp32_ros2_driver',
        name='esp32_ros2_driver'
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')
        ]),
        launch_arguments={
            'serial_port': '/dev/ttyUSB1',
            'frame_id': 'lidar_link'
        }.items()
    )

    return LaunchDescription([
        static_tf_node,
        esp32_node,
        lidar_launch
    ])
