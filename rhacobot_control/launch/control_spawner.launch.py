from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
 
    # Spawner for the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawner for the wheel controller
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawner for the tail controller
    tail_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tail_controller", "--controller-manager", "/controller_manager"],
    )


    # Event handler to spawn the wheel and tail controllers after the joint state broadcaster spawner exits
    wheel_and_tail_controllers_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[wheel_controller_spawner, tail_controller_spawner],
        )
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        wheel_and_tail_controllers_event_handler,
    ])