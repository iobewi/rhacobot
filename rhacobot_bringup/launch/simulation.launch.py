from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_rhacobot_simulation = get_package_share_directory("rhacobot_simulation")
    pkg_rhacobot_description = get_package_share_directory('rhacobot_description')
    pkg_rhacobot_localization = get_package_share_directory('rhacobot_localization')
    pkg_rhacobot_teleop = get_package_share_directory('rhacobot_teleop')
    pkg_rhacobot_navigation = get_package_share_directory('rhacobot_navigation')


    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_simulation, 'launch', 'simulation.launch.py'))
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_description, 'launch', 'visualization.launch.py'))
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_localization, 'launch', 'localization.launch.py'))
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_teleop, 'launch', 'teleop.launch.py')),
        launch_arguments={ 'gui': 'true' }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_navigation, 'launch', 'navigation.launch.py')),
        launch_arguments={ 'gui': 'true' }.items(),
    )


    return LaunchDescription([
        simulation,
        visualization,
        localization,
        teleop,
        navigation,
    ])