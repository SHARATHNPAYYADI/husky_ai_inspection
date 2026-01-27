# Husky AI Inspection (Simulation)

## Overview

This repository contains a **simulation-first inspection stack for the Clearpath Husky UGV**, built using **ROS 2, Gazebo, and Nav2**.  
The project focuses on creating a stable navigation and localization foundation in simulation, which will later be extended with **AI-based inspection capabilities**.

Development is intentionally incremental: navigation reliability first, intelligence next.

---

## About This Repository

This project is based on the open-source **Husky UGV ROS 2 stack** maintained by **Clearpath Robotics**.

Original repository:  
https://github.com/husky/husky (humble-devel)

This repository extends the upstream stack with:
- Custom Gazebo simulation environments
- Navigation and localization workflows
- Waypoint recording logic
- A future AI inspection pipeline (planned)

---

## Current Status

- Gazebo simulation with Husky: ✅ Working
- Custom construction-site world: ✅ Working
- Nav2 localization using a static map: ✅ Working
- Automatic initial pose setting: ✅ Working
- Navigation stack bring-up: ✅ Working
- Waypoint recording via RViz: ✅ Working

---

## Features

### Implemented

- Gazebo simulation with Clearpath Husky
- Custom construction-site world
- Nav2 localization using a pre-built map
- Automatic initial pose publisher
- Navigation stack bring-up
- Waypoint recording from RViz interactions

### In Progress / Planned

- AI-based inspection (vision / detection pipeline)
- SLAM tuning and localization improvements
- Autonomous navigation through recorded waypoints

---

## Repository Structure

```text
husky_ai_inspection/
├── husky_gazebo/              # Gazebo simulation and world files
├── husky_nav2/                # Nav2 maps, parameters, and configuration
├── husky_init_pose/           # Automatic initial pose node
├── husky_waypoint_recorder/   # Waypoint recording node
```

## Prerequisites

- ROS 2 (tested with **Humble / Jazzy**)
- Gazebo
- Nav2
- Clearpath Husky simulation packages
- `colcon` build tools

Make sure your ROS environment is sourced:

```bash
source /opt/ros/<ros-distro>/setup.bash
source ~/husky_ws/install/setup.bash
```

## Build Instructions

From your workspace root:

```bash
colcon build
source install/setup.bash
```

## Instructions

Follow the steps in order

### Step 1 – Launch Gazebo Simulation

```bash
ros2 launch husky_gazebo gazebo.launch.py world_path:=/home/sharathnpayyadi/husky_ws/src/husky_ai_inspection/husky_gazebo/worlds/construction_site.world
```

### Step 2 – Start Localization

```bash
ros2 launch nav2_bringup localization_launch.py   use_sim_time:=true   map:=/home/sharathnpayyadi/husky_ws/src/husky_ai_inspection/husky_nav2/maps/my_map.yaml params_file:=src/husky_ai_inspection/husky_nav2/config/nav2_params.yaml

```

### Step 3 – Set Initial Pose Automatically

```bash
ros2 run husky_init_pose auto_initial_pose
```

### Step 4 – Start Navigation Stack

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map:=src/husky_ai_inspection/husky_nav2/maps/my_map.yaml   params_file:=src/husky_ai_inspection/husky_nav2/config/nav2_params.yaml

```

### Step 5 – Launch RViz

```bash
rviz2
```

### Step 6 – Record Waypoints

```bash
ros2 run husky_waypoint_recorder follow_waypoints
```

## Demos
![Navigation Demo](videos/demo.mp4)