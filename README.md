# mrs_coverage_planner

A standalone ROS package for Multi-UAV Energy-Aware Coverage Path Planning (MCPP). This node serves as a wrapper coordinator for the underlying geometric decomposition and optimization library (`EnergyAwareMCPP`), exposing a clean ROS Service API (`/mrs_coverage_planner_node/compute_coverage_path`) to handle fleet mission tasking.

---

## Prerequisites & Dependencies

This package requires a working standard ROS Noetic environment alongside the Czech Technical University (CTU) Multi-robot Systems (MRS) system ecosystem:

```
# ROS Core Dependencies
sudo apt-get install ros-noetic-roscpp ros-noetic-std-msgs ros-noetic-geometry-msgs

# MRS Ecosystem Dependencies
ros-noetic-mrs-lib
ros-noetic-mrs-msgs

```

---

## Installation & Build Instructions

1. Clone or place this package structure into your active workspace source directory:
cd ~/your_workspace/src/
Place repository contents inside a folder named `mrs_coverage_planner`


2. Initialize and compile the workspace components using `catkin tools`:
cd ~/your_workspace
catkin build mrs_coverage_planner
source devel/setup.bash

---

## Configuration Setup

Node configurations are managed directly through local configuration blocks. Ensure parameters match your platform profile in `config/standalone_coverage_planner.yaml`:

```
# Global Projections Reference Origin
latitude_origin: 49.228330
longitude_origin: 15.225238
points_in_lat_lon: true

# Trajectory Sweep Parameters 
sweeping_step: 2            # Path offset interval between parallel lanes (meters)
number_of_rotations: 3      # Heading candidate pools checked by the optimization engine

# Drone Physical Profile Configurations
drone_mass: 3.2             # Mass used by the internal energy calculator model (kg)
propeller_radius: 0.19      # Structural dimension constraints (meters)
number_of_propellers: 4
average_acceleration: 2.0   # Operational bounds limit (m/s^2)
drone_area: 0.07            # Frontal area mapping cross-section (m^2)

```

---

## Execution & Verification Testing

### 1. Launching the Service Node

Spin up the main processing handler context using the provided standard initialization file:

```
roslaunch mrs_coverage_planner test_planner.launch

```

### 2. Programmatic Execution & Verification

A complete Python test tool pipeline (`visual_test.py`) is provided within the source repository directory structure to dynamically mock deployment request environments. It targets the advertised `/mrs_coverage_planner_node/compute_coverage_path` service, constructs valid nested array fields, and renders outputs in a visual format.

To run the verification test utility:

```
python3 visual_test.py

```