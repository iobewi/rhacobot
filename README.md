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

Clone the repository:
```bash
git clone https://github.com/ioio2995/rhacobot_ros2_control.git
cd rhacobot_ros2_control
```

Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the project:
```bash
colcon build
```

## Usage

To launch the robot:
```bash
ros2 launch bringup rhacobot_bringup.launch.py
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