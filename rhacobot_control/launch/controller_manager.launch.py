# Copyright 2024 Lionel ORCIL - github.com/ioio2995
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():

    pkg_share = get_package_share_directory("rhacobot_control")
    params_file = os.path.join(pkg_share, "config", "controllers.yaml")


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params_file],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    return LaunchDescription(
        [
            control_node
        ]
    )
