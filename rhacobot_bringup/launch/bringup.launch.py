import os


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    pkg_rhacobot_control = get_package_share_directory("rhacobot_control")
    pkg_rhacobot_description = get_package_share_directory("rhacobot_description")
    pkg_rhacobot_localization = get_package_share_directory("rhacobot_localization")
    pkg_rhacobot_teleop = get_package_share_directory("rhacobot_teleop")
    pkg_rhacobot_navigation = get_package_share_directory("rhacobot_navigation")

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_description, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={ 'use_simulation': 'false', 'use_controllers': 'true'  }.items(),
    )
    
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rhacobot_control, "launch", "control_manager.launch.py")
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rhacobot_description, "launch", "rviz.launch.py")
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rhacobot_localization, "launch", "localization.launch.py")
        )
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rhacobot_teleop, "launch", "teleop.launch.py")
        ),
        launch_arguments={"gui": "false"}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rhacobot_navigation, "launch", "navigation.launch.py")
        )
    )


    return LaunchDescription(
        [
            robot_state_publisher,
            control,
            rviz,
            teleop,
        ]
    )
