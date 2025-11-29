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


## Key Features

### 1. Robot Description (URDF)
<p align="center">
  <img src="images/rover.png" width="300">
</p>
- Defines the rover’s physical structure
- Includes IMU, Camera, Lidar.
- Viewable in RViz2
- Easily extendable for LiDAR, cameras, or GPS

### 2. IMU Integration
- `imu_sub` package subscribes to IMU topics
- Provides filtered orientation and acceleration
- Supports future sensor fusion

### 3. Map-Based Navigation
<p align="center">
  <img src="images/rviz.png" width="420">
  <img src="images/gazebotop.png" width="300">
</p>
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
- **ROS2** (Humble compatible)
- **Python 3**
- **C++**
- **URDF / XML**
- **Navigation2 Stack**
- **RViz2**

