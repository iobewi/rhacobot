import os
import xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    pkg_rhacobot_control = get_package_share_directory("rhacobot_control")
    pkg_rhacobot_description = get_package_share_directory('rhacobot_description')
    pkg_rhacobot_localization = get_package_share_directory('rhacobot_localization')
    pkg_rhacobot_teleop = get_package_share_directory('rhacobot_teleop')
    pkg_rhacobot_navigation = get_package_share_directory('rhacobot_navigation')

    xacro_file = os.path.join(pkg_rhacobot_description, 'urdf', "rhacobot.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={"use_simulation": "true", "controllers": "true"})
    robot_description_content = doc.toxml()

    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_control, 'launch', 'controller_manager.launch.py'))
    )

    controllers_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_control, 'launch', 'controllers_spawner.launch.py'))
    )
    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_description, 'launch', 'visualization.launch.py'))
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_localization, 'launch', 'localization.launch.py'))
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_teleop, 'launch', 'teleop.launch.py')),
        launch_arguments={ 'gui': 'false' }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_navigation, 'launch', 'navigation.launch.py'))
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': doc.toxml()}],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_manager,
                on_exit=[controllers_spawner],
            )
        ),
        visualization,
        localization,
        teleop,
        navigation,
    ])