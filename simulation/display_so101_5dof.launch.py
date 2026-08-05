#!/usr/bin/env python3
"""
Standalone RViz viewer for so101_5dof_calib.urdf.
Place this file in the SAME folder as the URDF and the assets/ directory, then:

    ros2 launch /full/path/to/display_so101_5dof.launch.py

Brings up robot_state_publisher + joint_state_publisher_gui + rviz2.
Relative 'assets/...' mesh paths are rewritten to absolute file:// URIs so RViz
can find them no matter what directory you launch from.
"""
import os
from urllib.parse import quote
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(here, "so101_5dof_calib.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Rewrite relative mesh paths -> absolute file:// URIs (URL-encoded so
    # spaces/parentheses in the folder name don't break resolution).
    abs_assets = f'filename="file://{quote(here)}/assets/'
    robot_description = robot_description.replace('filename="assets/', abs_assets)

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
        ),
    ])
