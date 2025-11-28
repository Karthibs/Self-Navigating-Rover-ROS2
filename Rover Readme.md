# Self-Navigating Rover (ROS2)

An autonomous ground-rover project built using **ROS2**, capable of **self-navigation, IMU-based state estimation, map-based localization, and waypoint following**. This repository contains the complete robot description, navigation setup, test scripts, and map files used for development and experiments.

---

## Project Overview

This project implements a **self-navigating rover** using the ROS2 ecosystem. It integrates:

- **Robot description (URDF)**
- **IMU sensor integration**
- **Navigation2 (Nav2) stack testing**
- **Map loading and waypoint navigation**
- **Custom rover control package**

The system is designed to run both on a **real rover** and simulation environments such as RViz or Gazebo.

---

## Repository Structure

```
Self-Navigating-Rover-ROS2/
│
├── imu_sub/                 # IMU subscription node (sensor interface)
│
├── rover_animal/
│   └── urdf/                # URDF model of the rover (robot description)
│
├── rover_master/            # Main rover control logic and ROS2 package
│
├── custom_map.pgm           # Occupancy grid maps
├── custom_map.yaml
├── field1.pgm
├── field1_2.pgm
├── field1_2.yaml
├── my_map.pgm
├── my_map.yaml
│
├── nav2_test.py             # Python script to test Navigation2 behaviours
│
└── build/                   # Auto-generated build files (colcon)
```

---

## Key Features

### 1. Robot Description (URDF)
- Defines the rover’s physical structure
- Includes links, joints, sensors
- Viewable in RViz2
- Easily extendable for LiDAR, cameras, or GPS

### 2. IMU Integration
- `imu_sub` package subscribes to IMU topics
- Provides filtered orientation and acceleration
- Supports future sensor fusion

### 3. Map-Based Navigation
Includes multiple `.pgm + .yaml` maps for testing:
- Indoor maps
- Field-like outdoor maps

Compatible with:
- `map_server`
- `nav2_amcl`
- Global & local planners

### 4. Nav2 Test Script
`nav2_test.py` demonstrates:
- Map loading
- Sending navigation goals
- Monitoring Nav2 feedback
- Useful for validating navigation pipeline

---

## Technologies Used
- **ROS2** (Foxy/Humble compatible)
- **Python 3**
- **C++**
- **URDF / XML**
- **Navigation2 Stack**
- **RViz2**

---

## How to Run

### 1. Clone the repository
```bash
git clone https://github.com/Karthibs/Self-Navigating-Rover-ROS2.git
cd Self-Navigating-Rover-ROS2
```

### 2. Source ROS2
```bash
source /opt/ros/humble/setup.bash
```

### 3. Build
```bash
colcon build
source install/setup.bash
```

### 4. Run IMU node
```bash
ros2 run imu_sub imu_subscriber
```

### 5. Launch Nav2 test
```bash
python3 nav2_test.py
```

---

## Robot Model (URDF)
The URDF located in `rover_animal/urdf/` defines:
- Rover chassis
- Wheel configuration
- IMU link
- Base transforms

This enables simulation, visualization, and integration with Nav2.

---

## Future Extensions
- LiDAR integration
- EKF sensor fusion
- SLAM (GMapping / Slam Toolbox)
- GPS / outdoor navigation
- Advanced motion planners

---

## Author
**B S Karthikeya Reddy**

---

## Purpose
This project demonstrates experience in:
- ROS2 architecture
- Autonomous navigation
- Sensor integration
- Robot modeling
- Real-world robotics development

It is part of my robotics portfolio for academic and industry applications.