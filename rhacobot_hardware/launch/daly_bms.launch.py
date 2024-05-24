from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():
    pkg_rhacobot_hardware = get_package_share_directory("rhacobot_hardware")
    bms_params_file = os.path.join(pkg_rhacobot_hardware, "config", "bms.yaml")
             
    bms_node=Node(
        package = 'dalybms_hardware',
        executable = 'dalybms_hardware',
        name="battery",
        parameters = [bms_params_file]
    )

    return LaunchDescription(
        [
           bms_node,
        ]
    )
