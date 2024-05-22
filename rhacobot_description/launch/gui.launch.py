import os
import xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_rhacobot_description = get_package_share_directory('rhacobot_description')

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_rhacobot_description, "launch" , "rviz.launch.py")]
        )
    )
    
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rhacobot_description, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={ 'use_simulation': 'false', 'use_controllers': 'false'  }.items(),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    return LaunchDescription(
        [
        robot_state_publisher,
        rviz,
        joint_state_publisher_node,
        ]
    )
