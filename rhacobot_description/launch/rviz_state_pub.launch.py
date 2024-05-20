import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    pkg_share_description = get_package_share_directory('rhacobot_description')

    xacro_file = os.path.join(pkg_share_description, 'urdf', "rhacobot.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_share_description, "launch/visualization.launch.py")]
        )
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': doc.toxml()}],
        condition=IfCondition(gui),
    )

    return LaunchDescription(
        [
        joint_state_publisher_node,
        robot_state_publisher_node,
        visualization,
        ]
    )
