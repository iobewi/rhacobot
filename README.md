# Rhacobot

This project implements the control of Rhacobot using ROS2. The project is organized into several components to facilitate development and integration.

## Directory Structure

- `bringup`: Scripts to launch various robot components.
- `control`: Robot control modules.
- `description`: Robot description (URDF, meshes).
- `navigation`: Configuration and scripts for navigation.
- `simulation`: Simulation files to test the robot in a virtual environment.
- `teleop`: Scripts for teleoperation.

## Prerequisites

- ROS2 (Dashing, Foxy, or Humble)
- Python 3.8+
- CMake 3.10+



## Installation

Prerequisites :
```bash
mkdir -p rhacobot_ws/src
cd rhacobot_ws
sudo rosdep init
```

Install script:
```bash
# curl
bash <(curl -s https://raw.githubusercontent.com/ioio2995/rhacobot/master/init_rhacobot.bash)
# or
# wget
bash <(wget -qO- https://raw.githubusercontent.com/ioio2995/rhacobot/master/init_rhacobot.bash)
```

## Usage

To launch the robot:
```bash
ros2 launch bringup bringup.launch.py
```

## Contribution

Contributions are welcome! Please submit pull requests or open issues to report problems.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Authors

- ioio2995

## Future Improvements

- Enhance documentation and add usage examples.
- Integrate unit and integration tests.