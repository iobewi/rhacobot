from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():

    pkg_share = get_package_share_directory("rhacobot_teleop")
    params_file = os.path.join(pkg_share, "config", "config.yaml")
    teleop_pub = "/wheel_controller/cmd_vel_unstamped"

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"deadzone": 0.05, "autorepeat_rate": 30.0}],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_wheel_joy_node",
        parameters=[params_file],
        remappings=[
            ("/cmd_vel", teleop_pub),
        ],
    )

    return LaunchDescription(
        [
            joy_node,
            teleop_node,
        ]
    )
