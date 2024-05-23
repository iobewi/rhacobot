from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():

    pkg_rhacobot_hardware = get_package_share_directory("rhacobot_hardware")
    wtgahrs1_params_file = os.path.join(pkg_rhacobot_hardware, "config", "wtgahrs1.yaml")
    wt31n_params_file = os.path.join(pkg_rhacobot_hardware, "config", "wt31n.yaml")
    wt901_params_file = os.path.join(pkg_rhacobot_hardware, "config", "wt901.yaml")
              
    wtgahrs1_node=Node(
        package = 'witmotion_ros',
        executable = 'witmotion_ros_node',
        parameters = [wtgahrs1_params_file]
    )

    wt901_node=Node(
        package = 'witmotion_ros',
        executable = 'witmotion_ros_node',
        parameters = [wt901_params_file]
    )

    wt31n_node=Node(
        package = 'witmotion_ros',
        executable = 'witmotion_ros_node',
        parameters = [wt31n_params_file]
    )

    return LaunchDescription(
        [
            wtgahrs1_node,

        ]
    )
