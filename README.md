# Intelligent Guide Robot - Webots Project

A robotic guide dog that follows another individual/robot in a domestic environment using Webots simulation.

## Features

- 🤖 **Rosbot** - Main guide robot with lidar, cameras, and IMU
- 🐢 **TurtleBot3** - Target robot to follow
- 🗺️ **SLAM Localization** - Real-time mapping and localization
- 🎯 **Following Behavior** - Rope-based following mechanism

## Quick Start

### Running the Simulation

1. **Open Webots:**
   ```bash
   /Applications/Webots.app/Contents/MacOS/webots worlds/domestic-environment.wbt
   ```

2. **Or use the launch script:**
   ```bash
   ./run_slam.sh standalone
   ```

### SLAM (Simultaneous Localization and Mapping)

The project includes SLAM capabilities:

- **Standalone SLAM** (No ROS2 required) - Currently active
- **ROS2 SLAM** (Advanced) - For full ROS2 integration

See [QUICK_START_SLAM.md](QUICK_START_SLAM.md) for details.

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

## Documentation

- [QUICK_START_SLAM.md](QUICK_START_SLAM.md) - Quick start for SLAM
- [SLAM_SETUP.md](SLAM_SETUP.md) - Detailed SLAM setup guide
- [README_COMPILATION.md](README_COMPILATION.md) - Compilation instructions

## Requirements

- **Webots** R2025a or later
- **Python 3.11+** (for controllers)
- **ROS2** (optional, for ROS2 SLAM integration)

## Building Controllers

```bash
./compile_from_terminal.sh
```

Or compile from Webots GUI: Right-click controller folder → "Make"

## SLAM Features

The standalone SLAM controller:
- ✅ Builds occupancy grid maps from lidar
- ✅ Tracks robot position using odometry
- ✅ Saves maps to JSON files
- ✅ Works without ROS2 installation
- ✅ Real-time mapping as robot moves

## Next Steps

1. **Integrate SLAM with following** - Use maps for navigation
2. **Visualize maps** - Create visualization tools for saved maps
3. **Add ROS2** - For advanced SLAM features (slam_toolbox)

## License

See individual file headers for license information.



