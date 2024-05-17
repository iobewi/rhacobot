# /bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share_description = get_package_share_directory("rhacobot_description")
    pkg_share_bringup = get_package_share_directory("rhacobot_bringup")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_bringup, "launch", "simulation.launch.py")
        )
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_description, "launch", "visualization.launch.py")
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_bringup, "launch", "localization.launch.py")
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_bringup, "launch", "navigation.launch.py")
        )
    )

    return LaunchDescription(
        [
            simulation,
            visualization,
            localization,
            navigation,
        ]
    )
