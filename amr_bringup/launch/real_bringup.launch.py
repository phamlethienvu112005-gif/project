"""Real Hardware Bringup - All drivers and data nodes for real AMR.

Launches:
  - Robot State Publisher (URDF from xacro)
  - ESP32 Hardware Bridge (serial -> ROS2)
  - CMD Velocity Splitter (cmd_vel -> diff_drive/cmd_vel + lift/cmd_vel)
  - LiDAR Driver (Sllidar A1M8)
  - Camera Depth Driver (PointCloud)
  - EKF Localization Node

Topic Flow:
  teleop/nav2 -> /cmd_vel -> cmd_vel_splitter -> /diff_drive/cmd_vel -> esp32_bridge -> Serial
                                       -> /lift/cmd_vel -> esp32_bridge -> Serial

TF Tree:
  map -> odom (from SLAM when running)
  odom -> base_footprint (from EKF)
  base_footprint -> base_link -> sensor_links/mechanical_links (from RSP)

Usage:
  ros2 launch amr_bringup real_bringup.launch.py camera_enable:=false
  ros2 launch amr_bringup real_bringup.launch.py lidar_port:=/dev/rplidar
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================
    # Launch Arguments
    # ============================================
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/rplidar',
        description='LiDAR serial port (fixed symlink via udev rules)'
    )
    lidar_port = LaunchConfiguration('lidar_port')

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='LiDAR serial baudrate'
    )
    serial_baudrate = LaunchConfiguration('serial_baudrate')

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='',
        description='LiDAR scan mode'
    )
    scan_mode = LaunchConfiguration('scan_mode')

    camera_enable_arg = DeclareLaunchArgument(
        'camera_enable',
        default_value='true',
        description='Enable camera driver (true/false)'
    )
    camera_enable = LaunchConfiguration('camera_enable')

    # ============================================
    # Package paths
    # ============================================
    pkg_amr_desc = get_package_share_directory('amr_description')
    pkg_amr_bringup = get_package_share_directory('amr_bringup')

    # URDF path
    xacro_file = os.path.join(pkg_amr_desc, 'urdf', 'amr.gazebo.xacro')
    ekf_config = os.path.join(pkg_amr_bringup, 'config', 'ekf.yaml')

    # Process xacro
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()

    # ============================================
    # Robot State Publisher
    # ============================================
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description_config
        }],
    )

    # ============================================
    # ESP32 Hardware Bridge
    # Receives: raw encoder ticks + gyro_z from ESP32
    # Publishes: /odom_raw, /imu/data_raw, /joint_states
    # Subscribes: /diff_drive/cmd_vel, /lift/cmd_vel, /esp32_cmd
    # ============================================
    esp32_bridge = Node(
        package='amr_hardware',
        executable='esp32_bridge',
        name='esp32_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'wheel_radius': 0.0325,    # Bán kính bánh xe 32.5mm
            'wheelbase': 0.472,          # Khoảng cách 2 bánh 472mm
        }],
    )

    # ============================================
    # CMD Velocity Splitter
    # Splits /cmd_vel into drive and lift commands
    # Inputs: /cmd_vel (from teleop or nav2)
    # Outputs: /diff_drive/cmd_vel, /lift/cmd_vel
    # ============================================
    cmd_vel_splitter = Node(
        package='amr_hardware',
        executable='cmd_vel_splitter',
        name='cmd_vel_splitter',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
    )

    # ============================================
    # LiDAR Driver (Sllidar A1M8)
    # ============================================
    lidar_launch = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'serial_port': lidar_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': scan_mode,
        }],
        remappings=[
            ('/scan', '/scan'),
        ],
    )

    # ============================================
    # Camera Depth Driver (Astra/Orbbec)
    # Publishes: /points (PointCloud2)
    # NOTE: Requires ros2_astra_camera or orbbec_camera package installed
    # ============================================
    camera_warning = LogInfo(
        msg='[amr_bringup] Camera driver: ros2_astra_camera package required',
        condition=UnlessCondition(camera_enable)
    )

    camera_driver = Node(
        package='ros2_astra_camera',
        executable='ros2_astra_camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'color_width': 640,
            'color_height': 480,
            'depth_width': 640,
            'depth_height': 480,
            'depth_registration': True,
        }],
        remappings=[
            ('/depth/points', '/points'),
        ],
        condition=IfCondition(camera_enable)
    )

    # ============================================
    # EKF Localization Node
    # Fuses: /odom_raw (wheel encoder) + /imu/data_raw (gyro_z)
    # Outputs: /odom (published TF: odom -> base_footprint)
    # ============================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            ekf_config
        ],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
    )

    # ============================================
    # Return LaunchDescription
    # ============================================
    return LaunchDescription([
        # Arguments
        lidar_port_arg,
        serial_baudrate_arg,
        scan_mode_arg,
        camera_enable_arg,

        # Robot State Publisher
        rsp,

        # Hardware Bridge (ESP32)
        esp32_bridge,

        # CMD Velocity Splitter
        cmd_vel_splitter,

        # LiDAR Driver
        lidar_launch,

        # Camera Depth Driver (conditional)
        camera_warning,
        camera_driver,

        # EKF Localization
        ekf_node,
    ])
