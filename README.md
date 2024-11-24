# MULTI-ROBOT MAP MERGING      [Code: MuRoMM]


![Screenshot from 2024-11-16 19-20-20](https://github.com/user-attachments/assets/2dde64ba-3bc4-45b3-98a7-5ec59935a6ba)


![Screenshot from 2024-11-19 18-59-31](https://github.com/user-attachments/assets/bc097d65-2ae6-4a29-b4a4-ef06c7e8e7ca)

![CICD Workflow status][![codecov](https://codecov.io/gh/Abhishek260101/Warehouse-Autonomous-Robot-System/graph/badge.svg?token=813CD16HJ6)](https://codecov.io/gh/Abhishek260101/Warehouse-Autonomous-Robot-System)

![CICD Workflow status](https://github.com/kashifansaricodes/Human-Obstacle-Detector-and-Tracker/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/kashifansaricodes/Human-Obstacle-Detector-and-Tracker/graph/badge.svg?token=T484S8WKBC)](https://codecov.io/gh/kashifansaricodes/Human-Obstacle-Detector-and-Tracker)[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Overview
This project implements a multi-robot mapping system using ROS2 Humble that enables two TurtleBot3 robots to explore and map an environment collaboratively. The system combines individual SLAM-generated maps from each robot to create a unified, comprehensive map of the environment.

### Key Features
- Frontier-based exploration algorithm for autonomous navigation
- Automatic map expansion and alignment
- Real-time map merging
- Multi-robot coordination
- Visualizations using RViz2

## Dependencies
- ROS2 Humble
- Ubuntu 22.04
- C++17 or higher
- OpenCV 4.5+
- TurtleBot3 packages for ROS2
- Nav2
- SLAM Toolbox
- tf2_ros

### Required ROS2 Packages
```bash
sudo apt update
sudo apt install ros-humble-turtlebot3
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-nav2*
sudo apt install ros-humble-tf2-ros
```

## Installation

### Build from Source
```bash
# Create a ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/Abhishek260101/Warehouse-Atonomous-Robot-System.git

# Install dependencies
sudo rosdep init  # if not initialized before
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the package
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

### Launch the System
1. Start the robots and their core functionalities:
```bash
ros2 launch multi_robot_exploration bringup_robots.launch.py
```

2. Launch SLAM for both robots:
```bash
ros2 launch multi_robot_exploration multi_slam.launch.py
```

3. Start the frontier exploration:
```bash
ros2 launch multi_robot_exploration frontier_exploration.launch.py
```

### Service Calls
To start the exploration:
```bash
ros2 service call /tb3_0_start std_srvs/srv/Empty
ros2 service call /tb3_1_start std_srvs/srv/Empty
```

### Visualization
Monitor the exploration progress in RViz2:
```bash
ros2 launch multi_robot_exploration rviz.launch.py
```

## Project Structure
```
multi_robot_exploration/
├── config/
│   ├── frontier_params.yaml
│   └── rviz_config.rviz
├── include/
│   └── multi_robot_exploration/
│       ├── front_expl.hpp
│       └── map_expansion.hpp
├── launch/
│   ├── bringup_robots.launch.py
│   ├── frontier_exploration.launch.py
│   └── multi_slam.launch.py
├── src/
│   ├── tb3_0_fe.cpp
│   ├── tb3_1_fe.cpp
│   └── map_expansion.cpp
├── test/
│   ├── test_frontier.cpp
│   └── test_map_expansion.cpp
└── package.xml
```

## Documentation
Generate documentation using Doxygen:
```bash
cd ~/ros2_ws/src/multi_robot_exploration
doxygen Doxyfile
```
Access the documentation at `docs/html/index.html`

## Testing
Run the tests:
```bash
cd ~/ros2_ws
colcon test
colcon test-result --verbose
```

Run code coverage:
```bash
colcon test --coverage
```

## Building With Different Types
By default, the workspace will be built in Release mode. You can specify the build type:
```bash
# Debug build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
# Release build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Contributing
1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Team
- [Team Member 1](https://github.com/username1)
- [Team Member 2](https://github.com/username2)

## References
- [ROS2 Navigation (Nav2)](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3 ROS2](https://github.com/ROBOTIS-GIT/turtlebot3/tree/ros2)

## Known Issues
- Map alignment may need manual adjustment in certain scenarios
- Frontier detection sensitivity might need tuning based on environment
- Transform lookups may timeout in large environments

## Future Work
- Implementation of more sophisticated map merging algorithms
- Addition of loop closure detection
- Integration with 3D mapping capabilities
- Enhanced multi-robot coordination strategies
- Implementation of lifecycle nodes
- Addition of parameter services
- Integration with ROS2 composable nodes

## ROS2-Specific Features
This implementation takes advantage of ROS2 features including:
- Lifecycle nodes for better state management
- Quality of Service (QoS) settings for reliable communication
- Parameter services for dynamic reconfiguration
- Action servers for long-running tasks
- Modern C++ features and ROS2 best practices

## Environment Variables
Make sure to set these environment variables:
```bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

## Acknowledgments
- ROS2 Community
- TurtleBot3 Development Team
- [Your University/Organization Name]

## Troubleshooting
Common issues and solutions:
1. Transform timeout errors:
   - Increase transform timeout in parameter files
2. Navigation issues:
   - Check Nav2 parameters
   - Verify map resolution
3. SLAM issues:
   - Adjust SLAM Toolbox parameters
   - Check sensor data
