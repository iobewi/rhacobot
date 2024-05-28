#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rhacobot_localization = get_package_share_directory("rhacobot_localization")
  
    ekf_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        respawn=True,
        parameters=[os.path.join(pkg_rhacobot_localization, "config", "ekf_3d.yaml")],
        remappings=[
            ("/odometry/filtered", "/odom"),
        ],
    )
        
    return LaunchDescription([
        ekf_localization_node
    ])
