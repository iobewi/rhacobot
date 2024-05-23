#!/bin/bash
# Bash script for setup rhacobot env
# Author: Lionel ORCIL - github.com/ioio2995
#
# Specify the path of the directory to check
src_dir="./src"

# Check if the directory exists
if [ ! -d "$src_dir" ]; then
    # If the directory does not exist, create it
    mkdir -p "$src_dir"
    echo "The directory $src_dir has been created."
else
    echo "The directory $src_dir already exists."
fi

# Import repositories using here document
vcs import ./$src_dir <<EOL
repositories:
    myactuator/myactuator_ros2_control:
        type: git
        url: https://github.com/ioio2995/myactuator_ros2_control.git
        version: master
    myactuator/myactuator_rmd:
        type: git
        url: https://github.com/2b-t/myactuator_rmd.git
        version: main
    rhacobot:
        type: git
        url: https://github.com/ioio2995/rhacobot.git
        version: master
    witmotion_ros:
        type: git
        url: https://github.com/ioio2995/witmotion_IMU_ros.git
        version: ros2-ioio
    witmotion_ros/witmotion-uart-qt:
        type: git
        url: https://github.com/ElettraSciComp/witmotion_IMU_QT.git
        version: main
EOL

# Update rosdep
rosdep update --rosdistro=$ROS_DISTRO

# Update the package list and install dependencies
sudo apt update
rosdep install --from-paths ./$src_dir --ignore-src -r -y

# Install by symlink
colcon build --symlink-install 