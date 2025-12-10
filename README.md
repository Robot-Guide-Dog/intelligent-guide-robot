# Intelligent Guide Robot - Group 44

A simulated robotic guide dog that follows a human/robot target and navigates a domestic environment in Webots.

## Our Implementations

- Particle-filter SLAM in Python (`controllers/rosbot/rosbot.py`):
  - Differential drive odometry, motion/measurement models, log-odds occupancy grid, systematic resampling.
  - LiDAR likelihoods and free/occupied map updates, JSON map scan exports (e.g., `controllers/rosbot/slam_map_*.json`).
  - LiDAR forward-cone checks, side-distance steering, reverse/spin escape, corner-escape fallback.
- Human-following via vision + depth:
  - Hybrid HSV/RGB green-segmentation, median depth sampling on the Astra depth camera to regulate following distance.
- Supervisor following logic:
  - Rope-based following controller and TurtleBot3 target motion for dynamic user behaviour.
- World setup:
  - Custom `domestic-environment.wbt` world with household layouts.

## Pre-programmed Aspects/Packages

- Rosbot/TurtleBot3 robot models.
- Webots controller APIs for sensors/actuators (camera, depth, LiDAR, wheel encoders, motors).
- Standard Python libraries only; no external SLAM or Computer Vision frameworks (e.g no SLAM Toolbox or OpenCV/YOLO).

## Attribution

- SLAM, odometry, mapping, and avoidance: Kevin Titus.
- Vision-based user detection and depth estimation: Sharifah Syed Yazid.
- 

## Repository layout

```
intelligent-guide-robot/
├── controllers/
│   ├── rosbot/                 # Particle-filter SLAM + avoidance + follow
│   ├── rosbot_slam/            # SLAM controller variant
│   ├── rosbot_slam_standalone/ # Non-ROS2 SLAM controller
│   ├── turtlebot3_ostacle_avoidance/ # Target robot motion
│   └── rope_supervisor/        # Following supervisor
├── worlds/
│   └── domestic-environment.wbt    # Main simulation world
├── ros2_ws/
    └── src/webots_slam/            # Optional ROS2 package
```

## Running the simulation (Webots)

1) Open `worlds/domestic-environment.wbt` in Webots R2025a.  
2) Set the ROSbot controller to `rosbot` (particle-filter SLAM)  
3) Set the TurtleBot3 to `turtlebot3_ostacle_avoidance` to act as the moving user target.  
4) Optionally enable `rope_supervisor` for the following behaviour.  
5) Run the simulation; maps are saved to `controllers/rosbot/slam_map_*.json`.

## Key behaviours

- SLAM: builds occupancy grids from LiDAR, localises via particle filter, exports JSON maps.
- Obstacle avoidance: reverse/spin/steer/corner-escape based on LiDAR ranges.
- Human-following: green-segmentation + depth to maintain distance to the target.


