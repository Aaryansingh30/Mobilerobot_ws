This repository contains my workspace for mobile robot projects, including custom **EKF-SLAM**, **SLAM Toolbox**, **Nav2 autonomous navigation**, and **URDF modeling** for simulation in Gazebo.

# ***Features***
**Custom EKF-SLAM Implementation:** Developed from scratch in C++ for mobile robot mapping and localization.
**SLAM Toolbox Integration:** Implementation using slam_toolbox for real-time mapping.
**Autonomous Navigation:** Configured nav2_bringup for path planning and obstacle avoidance.
**URDF Mobile Robot Model:** Designed a detailed URDF model for simulation in Gazebo.

# 1. ***Simulating the Mobilerobot***
- For visualising the mobilerobot only in rviz : 
```bash
ros2 launch mobilerobot_description mobilerobot_rviz.launch.py
```
- To spawn the mobilerobot in an empty gazebo world:
```bash
ros2 launch mobilerobot_description mobilerobot_spawn.launch.py 
```
- To spawn the mobilerobot in a custom world:
```bash
ros2 launch mobilerobot_description mobilerobot_world.launch.py 
```

# 2. ***Custom EKF-SLAM***
- To launch the EKF-SLAM along with the simulation:
```bash
ros2 launch ekf_slam ekf_slam_launch.launch.py 
```

# 3. ***SLAM Toolbox and NAV2_bringup for Autonomous Navigation***
- To launch the SLAM Toolbox and NAV2_bringup along with the simulation of the mobilerobot:
```bash
ros2 launch mobilerobot_nav2_stack mobilerobot_nav2_stack_launch.launch.py 
 ```
```bash
Mobilerobot_ws/
├── src/
│   ├── ekf_slam/                 # Custom EKF-SLAM implementation
│   ├── mobilerobot_nav2_stack/   # For implementing autonomous navigation using SLAM Toolbox and NAV2_bringup
│   ├── mobilerobot_description/  # URDF model for Gazebo simulation
└── README.md
```
