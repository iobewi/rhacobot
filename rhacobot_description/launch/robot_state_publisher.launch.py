import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory


# evaluates LaunchConfigurations in context for use with xacro.process_file(). Returns a list of launch actions to be included in launch description
def robot_description(context, *args, **kwargs):
    use_simulation = LaunchConfiguration('use_simulation').perform(context)
    use_controllers = LaunchConfiguration('use_controllers').perform(context)
    model_path = LaunchConfiguration('model_path').perform(context)
    
    robot_description_config = xacro.process_file(model_path, mappings={"use_simulation": use_simulation, "use_controllers": use_controllers})
    robot_description_content = robot_description_config.toxml()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}])

    return [robot_state_publisher_node]

def generate_launch_description():
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='Whether to use simulation mode'
    )
    
    use_controllers_arg = DeclareLaunchArgument(
        'use_controllers',
        default_value='false',
        description='Whether to use controllers'
    )

    pkg_rhacobot_description = get_package_share_directory('rhacobot_description')
    
    model_path_arg = DeclareLaunchArgument(
      'model_path',
      default_value=os.path.join(pkg_rhacobot_description, 'urdf', "rhacobot.urdf.xacro"),
      description='path to urdf'
    )
    
    robot_state_publisher_node = OpaqueFunction(function=robot_description) 
    
    return LaunchDescription(
        [
        use_simulation_arg,
        use_controllers_arg,
        model_path_arg,
        robot_state_publisher_node,
        ]
    )
