# Intelligent Guide Robot - Webots Project

A robotic guide dog that follows another individual/robot in a domestic environment using Webots simulation.

## Features

- 🤖 **Rosbot** - Main guide robot with lidar, cameras, and IMU
- 🐢 **TurtleBot3** - Target robot to follow
- 🗺️ **SLAM Localization** - Real-time mapping and localization
- 🎯 **Following Behavior** - Rope-based following mechanism

## Quick Start


### SLAM (Simultaneous Localization and Mapping)

The project includes SLAM capabilities:

### Controllers

- `rosbot_slam_standalone` - SLAM without ROS2 (current)
- `rosbot_slam` - SLAM with ROS2 integration
- `rosbot` - Original controller
- `turtlebot3_ostacle_avoidance` - Target robot controller
- `rope_supervisor` - Following behavior supervisor

## Project Structure

```
intelligent-guide-robot/
├── controllers/
│   ├── rosbot_slam/          # SLAM controllers
│   ├── rosbot/                # Original rosbot controller
│   ├── turtlebot3_ostacle_avoidance/
│   └── rope_supervisor/       # Following behavior
├── worlds/
│   └── domestic-environment.wbt  # Main simulation world
├── ros2_ws/                   # ROS2 workspace (optional)
│   └── src/webots_slam/       # ROS2 SLAM package
├── run_slam.sh                # Launch script
└── QUICK_START_SLAM.md        # SLAM quick start guide
```

## SLAM Features

The standalone SLAM controller:
- ✅ Builds occupancy grid maps from lidar
- ✅ Tracks robot position using odometry
- ✅ Saves maps to JSON files
- ✅ Works without ROS2 installation
- ✅ Real-time mapping as robot moves


