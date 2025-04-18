# Pure Pursuit ROS2 Package

This repository provides a ROS2 implementation of the Pure Pursuit controller for F1TENTH simulation and real robots. 


## Quick Start

### 1. Build the Package

```bash
cd ~/ros_ws
colcon build 
source install/setup.bash
```

### 2. Run the C++ Pure Pursuit Node

```bash
ros2 run pure_pursuit pure_pursuit_node
```

### 3. Simulation Requirements

Make sure your simulator or robot is publishing to the `/ego_racecar/odom` topic.

The node will automatically load waypoints from the installed `levine2.csv` file.

### 4. Visualization

Visualize waypoints and drive commands in RViz2 by adding the following topics:

- `/waypoints_markers`
- `/drive`
