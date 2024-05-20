import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory("rhacobot_bringup")
    position_goals = os.path.join(pkg_share, 'config', 'joint_trajectory_publisher.yaml')

    return LaunchDescription(
        [
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="publisher_joint_trajectory_controller",
                parameters=[position_goals],
                output="both",
            )
        ]
    )