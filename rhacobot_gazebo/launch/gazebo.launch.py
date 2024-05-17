from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 

import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory


def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rhacobot_description"), "urdf", "rhacobot.urdf.xacro"]
            ),
            " ",
            "use_gazebo:=true",
            " ",
            "controllers:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    pkg_gazebo_ros = get_package_share_path('gazebo_ros')

    gazebo_path = os.path.join(get_package_share_path('rhacobot_bringup'),
                             'config', 'gazebo.config.yaml')

    pkg_share_control = get_package_share_directory('rhacobot_control')

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share_control, 'launch/controllers.launch.py'))
    )

    pkg_share_description = get_package_share_directory('rhacobot_description')

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_share_description, 'launch/visualization.launch.py'))
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
                launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_path }.items()
         )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description]
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "rhacobot"]
    )


    robot_steering_node = Node (
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        parameters=[{"default_topic": "/wheel_controller/cmd_vel_unstamped"}] 
    )

    return LaunchDescription([
        controllers,
        robot_state_publisher_node,
        gazebo,
        spawn_entity_node,
        visualization,
        robot_steering_node,
    ])