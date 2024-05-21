from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_controller", "--controller-manager", "/controller_manager"],
    )

    tail_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tail_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[wheel_controller_spawner, tail_controller_spawner],
                )
            ),
        ]
    )
