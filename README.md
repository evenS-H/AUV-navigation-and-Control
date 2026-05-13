# AUV Navigation and Control

ROS 2 codebase developed for a bachelor thesis on autonomous underwater vehicle (AUV) navigation and control. The system implements a complete Guidance, Navigation and Control (GNC) stack with 3D waypoint path following using Line-of-Sight (LOS) guidance, discrete PID control, and sensor fusion from DVL, pressure sensor and compass.

## Contents

- **DVL** — TCP-based DVL A50 interface, publishes velocity to ROS 2 topics
- **Pressure** — Bar30 pressure sensor interface via MAVLink/MAVROS
- **Navigation** — DVL + pressure + compass fusion to NED odometry
- **LOS** — 3D Line-of-Sight guidance (horizontal and vertical plane)
- **PID** — Fuzzy-gain PID controller for yaw and pitch
- **Thruster_Vectoring** — Maps control output to thruster and servo commands via MAVROS RC override
- **Mission_Manager** — Waypoint sequencer and mission state machine
- **World_Click_Map** — Leaflet map HTTP server for interactive waypoint selection
- **auv_bringup** — Launch files and configuration for the full AUV stack

## Requirements

- ROS 2 Jazzy
- Python 3.12
- MAVROS
- BlueOS / ArduPilot setup
- numpy
- pymap3d
- pymavlink

## Build

```bash
cd ws
colcon build
source install/setup.bash
```

## Run

```bash
# Full system
ros2 launch auv_bringup full_system.launch.py

# Sensors only
ros2 launch auv_bringup sensor_only.launch.py
```

## Archive

The submitted version is archived on Zenodo and cited in the bachelor thesis.
