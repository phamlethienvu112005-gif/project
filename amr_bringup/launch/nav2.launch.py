"""Nav2 Launch - Navigation2 with A* Planner + MPPI Controller.

Requires:
  - /odom topic (from EKF)
  - /scan topic (from LiDAR)
  - /map topic (from SLAM or static map)

Subscribes:
  - /odom
  - /scan
  - /map

Publishes:
  - /cmd_vel (to ESP32 bridge)

TF Tree after launch:
  map -> odom (SLAM or AMCL)
  odom -> base_footprint (EKF)
  base_footprint -> base_link -> ...

Usage:
  ros2 launch amr_bringup nav2.launch.py
  ros2 launch amr_bringup nav2.launch.py map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import IncludeLaunchDescription


def generate_launch_description():
    # ============================================
    # Launch Arguments
    # ============================================
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map YAML file (empty = SLAM mode)'
    )

    # ============================================
    # Package paths
    # ============================================
    pkg_amr_nav = get_package_share_directory('amr_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Default params file
    default_params = os.path.join(pkg_amr_nav, 'config', 'nav2_params.yaml')

    # Update launch argument default
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to nav2_params.yaml'
    )

    # ============================================
    # Nav2 Bringup (official)
    # ============================================
    nav2_bringup_path = os.path.join(
        pkg_nav2_bringup,
        'launch', 'bringup_launch.py'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_path),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
        }.items()
    )

    # ============================================
    # Return LaunchDescription
    # ============================================
    return LaunchDescription([
        map_arg,
        params_file_arg,
        nav2_bringup,
    ])
