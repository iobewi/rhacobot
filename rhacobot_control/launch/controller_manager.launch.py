from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():

    pkg_rhacobot_control = get_package_share_directory("rhacobot_control")
    params_file = os.path.join(pkg_rhacobot_control, "config", "controllers.yaml")


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params_file],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    return LaunchDescription(
        [
            control_node
        ]
    )
