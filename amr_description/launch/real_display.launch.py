"""Launch AMR robot for real hardware visualization in RViz.

This launch file is designed for the physical robot where joint_states
are published by ESP32/micro-ROS hardware (not simulated).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("amr_description")

    # Path to XACRO file
    xacro_file = os.path.join(pkg_share, "urdf", "amr.gazebo.xacro")

    # Check if XACRO file exists
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"XACRO file not found: {xacro_file}")

    # Parse XACRO file
    try:
        from xacro import parse, process_doc
        doc = parse(open(xacro_file))
        process_doc(doc)
        robot_description_content = doc.toxml()
    except Exception as e:
        raise RuntimeError(f"Failed to parse XACRO file '{xacro_file}': {e}")

    # RViz config path
    default_rviz_config = os.path.join(pkg_share, "rviz", "robot.rviz")

    return LaunchDescription([
        # Log info about the configuration
        LogInfo(msg=[
            "Launching AMR Real Robot Visualization",
            " - XACRO: ", xacro_file,
            " - use_sim_time: false (real hardware)"
        ]),

        # Launch Argument: Enable/Disable RViz
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Whether to launch RViz2.",
        ),

        # Launch Argument: RViz config file
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Path to RViz configuration file.",
        ),

        # Node: robot_state_publisher
        # Publishes TF transforms based on robot_description and joint_states
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": False,  # Real hardware uses wall-clock time
                "robot_description": robot_description_content,
            }],
        ),

        # Node: rviz2
        # Visualizes the robot model and TF tree
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{
                "use_sim_time": False,
            }],
        ),
    ])
