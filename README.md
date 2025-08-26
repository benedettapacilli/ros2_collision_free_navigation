[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
![ROS2-Humble](https://img.shields.io/badge/ROS2-Humble-blue)


# TurtleBot3 Simple Navigator

A ROS2 package implementing a simple autonomous navigation behavior for TurtleBot3 robots. This project demonstrates basic obstacle avoidance and wandering behaviors using laser scan data.

## Overview

The `SimpleNavigator` node creates a wandering robot that:
- Moves forward while avoiding obstacles
- Uses laser scan data for environment perception
- Implements a state machine with forward movement and turning behaviors
- Adds random "tumbling" for exploration variety
- Features omnidirectional obstacle avoidance with configurable parameters

## Requirements

- ROS2 (tested with Humble)
- Gazebo simulation environment
- TurtleBot3 packages
- C++ compiler

## Dependencies

- `rclcpp` - ROS2 C++ client library
- `sensor_msgs` - For LaserScan messages
- `geometry_msgs` - For Twist (velocity) commands
- `nav_msgs` - Navigation message types
- `tf2` - Transform library
- `visualization_msgs` - For visualization (if needed)

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/your_ros2_workspace/src
git clone [<your-repo-url>](https://github.com/benedettapacilli/ros2_collision_free_navigation)
```

2. Build the package:
```bash
cd ~/your_ros2_workspace
colcon build --packages-select turtlebot3_simple_navigator
source install/setup.bash
```

## Usage

### Basic Launch

1. Start the TurtleBot3 simulation (make sure you have `TURTLEBOT3_MODEL` set, I tested on the burger robot, you can try others):
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. Run the simple navigator:
```bash
ros2 launch turtlebot3_simple_navigator simple_navigator.launch.py
```

### With Custom Parameters

You can override default parameters using the provided configuration file:
```bash
ros2 run turtlebot3_simple_navigator simple_navigator --ros-args --params-file install/turtlebot3_simple_navigator/share/turtlebot3_simple_navigator/config/params.yaml
```

Or set individual parameters:
```bash
ros2 run turtlebot3_simple_navigator simple_navigator --ros-args -p forward_speed:=0.3 -p safety_distance:=0.25
```

## Parameters

The node accepts the following parameters:

### Motion Parameters
- `forward_speed` (double, default: 0.2): Forward movement speed in m/s
- `turn_speed` (double, default: 0.8): Turning speed in rad/s
- `safety_distance` (double, default: 0.4): Minimum safe distance to obstacles in meters
- `front_angle_range` (double, default: 0.5): Angular range considered as "front" in radians

### Wandering Behavior
- `wander_std` (double, default: 0.1): Standard deviation for random movement noise
- `tumble_mean_interval` (double, default: 5.0): Average time between random tumbles in seconds
- `tumble_max_angle` (double, default: π/6): Maximum angle for random tumbles in radians

### Avoidance Behavior
- `influence_distance` (double, default: 0.8): Distance at which obstacles start influencing movement
- `avoid_gain` (double, default: 1.2): Gain factor for obstacle avoidance strength

## Algorithm Description

### State Machine
The robot operates in two states:
1. **MOVING_FORWARD**: Default state where the robot moves forward while avoiding obstacles
2. **TURNING**: Triggered when obstacles are detected or during tumbling behavior

### Obstacle Avoidance
The implementation uses an omnidirectional avoidance algorithm that:
- Scans all laser rays within the influence distance
- Computes a repulsive "torque" based on obstacle proximity
- Uses the formula: `torque += -weight * sin(angle)` where `weight = (1/range) - (1/influence_distance)`
- Applies the torque with configurable gain to the angular velocity

### Wandering Logic
- Adds Gaussian noise to movement for natural wandering
- Implements exponential distribution for tumble timing
- Early exit from turns when the path is clear

## Topics

### Subscribed
- `/scan` (sensor_msgs/LaserScan): Laser scan data for obstacle detection

### Published
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot movement

## Useful Links

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)  
- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)  
- [Gazebo Documentation](https://gazebosim.org/docs)  
