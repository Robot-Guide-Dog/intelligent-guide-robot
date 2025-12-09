# Visual Diagrams and Flow Charts

## System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    WEBOTS SIMULATION                            │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ Physical Environment                                       │ │
│  │ ├─ Walls, furniture, obstacles                            │ │
│  │ ├─ ROSbot (guide robot with sensors)                      │ │
│  │ ├─ TurtleBot3 (follower robot)                            │ │
│  │ └─ Rope connecting them (physics-based)                   │ │
│  └────────────────────────────────────────────────────────────┘ │
│                          │                                       │
│                          ↓ (sensor data)                        │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ ROSbot Controller (rosbot.py)                              │ │
│  │                                                             │ │
│  │  ┌──────────────────────────────────────────────────┐     │ │
│  │  │ INPUT PROCESSING                                │     │ │
│  │  ├─ LIDAR: 360° scans (640 points)                 │     │ │
│  │  ├─ Camera RGB: 640×480 pixels                     │     │ │
│  │  ├─ Camera Depth: 640×480 depth map                │     │ │
│  │  ├─ Wheel encoders: 4 position sensors            │     │ │
│  │  ├─ IMU: accelerometer, gyro, compass             │     │ │
│  │  └─ Distance sensors: 4 ultrasonic                │     │ │
│  │  └──────────────────────────────────────────────────┘     │ │
│  │                          │                                  │ │
│  │                          ↓                                  │ │
│  │  ┌──────────────────────────────────────────────────┐     │ │
│  │  │ SLAM MODULE (Particle Filter)                    │     │ │
│  │  ├─ 100 particles track robot position             │     │ │
│  │  ├─ LIDAR updates occupancy grid                   │     │ │
│  │  ├─ Odometry predicts motion                       │     │ │
│  │  ├─ Result: Occupancy grid + Robot pose           │     │ │
│  │  └──────────────────────────────────────────────────┘     │ │
│  │                          │                                  │ │
│  │        (Data: 200×200 grid, robot position)                │ │
│  │                          │                                  │ │
│  │                          ↓                                  │ │
│  │  ┌──────────────────────────────────────────────────┐     │ │
│  │  │ BEHAVIOR SELECTOR                               │     │ │
│  │  ├─ User detection (RGB camera)?                   │     │ │
│  │  │   YES → Use follow_motor_speeds()               │     │ │
│  │  │   NO → Continue                                 │     │ │
│  │  ├─ Path planned?                                  │     │ │
│  │  │   YES → Use waypoint_motor_speeds()             │     │ │
│  │  │   NO → Continue                                 │     │ │
│  │  └─ Default: Use compute_motor_speeds()            │     │ │
│  │  └──────────────────────────────────────────────────┘     │ │
│  │                          │                                  │ │
│  │                          ↓                                  │ │
│  │  ┌─────────────────────────────────────┐                   │ │
│  │  │ USER FOLLOWING (if active)          │                   │ │
│  │  ├─ Detect green person in RGB image   │                   │ │
│  │  ├─ Get distance from depth camera     │                   │ │
│  │  └─ Calculate speed to maintain distance                   │ │
│  │  └─────────────────────────────────────┘                   │ │
│  │           OR                                               │ │
│  │  ┌─────────────────────────────────────┐                   │ │
│  │  │ WAYPOINT FOLLOWING (if path exists) │                   │ │
│  │  ├─ [NEW] Get next waypoint            │                   │ │
│  │  ├─ [NEW] Calculate angle to waypoint  │                   │ │
│  │  ├─ [NEW] Apply proportional steering  │                   │ │
│  │  ├─ [NEW] Check for obstacles         │                   │ │
│  │  └─ [NEW] Advance waypoint if reached │                   │ │
│  │  └─────────────────────────────────────┘                   │ │
│  │           OR                                               │ │
│  │  ┌─────────────────────────────────────┐                   │ │
│  │  │ EXPLORATION (default)               │                   │ │
│  │  ├─ Detect front obstacle              │                   │ │
│  │  ├─ Avoid obstacles                    │                   │ │
│  │  └─ Continue mapping environment       │                   │ │
│  │  └─────────────────────────────────────┘                   │ │
│  │                          │                                  │ │
│  │                          ↓                                  │ │
│  │  ┌──────────────────────────────────────────────────┐     │ │
│  │  │ PATH PLANNER (if navigation_goal is set)        │     │ │
│  │  ├─ [NEW] A* algorithm                             │     │ │
│  │  ├─ [NEW] Search occupancy grid                    │     │ │
│  │  ├─ [NEW] Return waypoint list                     │     │ │
│  │  └──────────────────────────────────────────────────┘     │ │
│  │                          │                                  │ │
│  │                          ↓                                  │ │
│  │  ┌──────────────────────────────────────────────────┐     │ │
│  │  │ OUTPUT: Motor Commands                           │     │ │
│  │  ├─ Left motor speed  (m/s)                        │     │ │
│  │  ├─ Right motor speed (m/s)                        │     │ │
│  │  └─ 4 wheels: front-left, front-right,             │     │ │
│  │     rear-left, rear-right                          │     │ │
│  │  └──────────────────────────────────────────────────┘     │ │
│  │                                                             │ │
│  └────────────────────────────────────────────────────────────┘ │
│                          │                                       │
│                          ↓ (motor commands)                     │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ Physics Engine                                            │ │
│  ├─ Apply motor forces to wheels                             │ │
│  ├─ Calculate robot movement                                │ │
│  ├─ Rope supervisor: spring-damper forces                   │ │
│  ├─ TurtleBot3 pulled by rope                               │ │
│  └─ Return updated positions for next iteration             │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## A* Algorithm Flowchart

```
┌─────────────────────────────────────┐
│ START: plan_path()                  │
│ Input: start, goal, occupancy_grid  │
└────────────┬────────────────────────┘
             │
             ↓
┌─────────────────────────────────────┐
│ Convert world coordinates to grid   │
│ start_grid = world_to_grid(start)   │
│ goal_grid = world_to_grid(goal)     │
└────────────┬────────────────────────┘
             │
             ↓
┌─────────────────────────────────────┐
│ Is start cell valid?                │
│ (not occupied, in bounds)           │
└───────┬─────────────────────────┬───┘
        │ NO                      │ YES
        ↓                         ↓
    RETURN []              ┌─────────────────┐
    (no path)              │ Is goal valid?  │
                           └───┬──────────┬──┘
                               │ NO       │ YES
                               ↓         ↓
                           RETURN []  ┌─────────────────────┐
                           (no path)  │ Initialize A*:      │
                                      │ open_set = [start]  │
                                      │ g_score = 0 at start│
                                      │ f_score = h at start│
                                      └──────────┬──────────┘
                                                 │
                                                 ↓
                                      ┌──────────────────────┐
                                      │ While open_set:      │
                                      └────────┬─────────────┘
                                               │
                                               ↓
                    ┌──────────────────────────────────────────┐
                    │ Pick cell with lowest f_score            │
                    │ current = heappop(open_set)              │
                    └────────────────────┬─────────────────────┘
                                         │
                                         ↓
                    ┌──────────────────────────────────────────┐
                    │ Is current == goal_grid?                 │
                    └─────┬──────────────────────┬──────────────┘
                          │ NO                   │ YES
                          ↓                      ↓
                    ┌──────────────────┐  ┌─────────────────┐
                    │ Add to closed_set│  │ reconstruct_path│
                    │ and process      │  └────────┬────────┘
                    │ neighbors        │           │
                    └────────┬─────────┘           ↓
                             │            ┌──────────────────┐
                    ┌────────▼────────┐   │ Return path      │
                    │ For each neighbor│   │ (world coords)   │
                    │ of current       │   └────────┬─────────┘
                    └────────┬────────┘            │
                             │                     │
                    ┌────────▼─────────────────┐   │
                    │ Is neighbor in closed_set?   │
                    └─────┬──────────────────┬─┘   │
                          │ YES              │ NO  │
                          │                  ↓     │
                          │         ┌─────────────────┐
                          │         │ Calculate g     │
                          │         │ tentative_g =   │
                          │         │ g[current] + 1  │
                          │         └────────┬────────┘
                          │                  │
                          │         ┌────────▼──────────────┐
                          │         │ Is path through here  │
                          │         │ better than before?   │
                          │         └──┬──────────────────┬─┘
                          │            │ NO               │ YES
                          │            │                  ↓
                          │            │        ┌──────────────────┐
                          │            │        │ Update g_score   │
                          │            │        │ Update f_score   │
                          │            │        │ Add to open_set  │
                          │            │        └────────┬─────────┘
                          │            │                 │
                          └────────────┴─────────────────┘
                                       │
                                       ↓
                    ┌──────────────────────────────────┐
                    │ Back to: Pick next lowest f_score│
                    └──────────────────────────────────┘
                             (loop)

(loop continues until goal found or open_set empty)

┌─────────────────────────────────────┐
│ If open_set empty and goal not found│
│ RETURN [] (no path exists)          │
└─────────────────────────────────────┘
```

---

## Waypoint Following Control Loop

```
┌────────────────────────────────────────────────────────┐
│ START: compute_waypoint_motor_speeds()                 │
└───────────────────┬────────────────────────────────────┘
                    │
                    ↓
        ┌───────────────────────┐
        │ Is path empty or      │
        │ at end?               │
        └─────┬───────────────┬─┘
              │ YES           │ NO
              ↓               ↓
         RETURN [0,0]  ┌─────────────────┐
         (stop)        │ Get waypoint    │
                       │ from path list  │
                       └────────┬────────┘
                                │
                                ↓
                   ┌────────────────────────┐
                   │ Calculate distance:    │
                   │ dx = waypoint.x - pos.x│
                   │ dy = waypoint.y - pos.y│
                   │ dist = sqrt(dx² + dy²) │
                   └────────────┬───────────┘
                                │
                                ↓
                   ┌────────────────────────┐
                   │ Is dist <              │
                   │ waypoint_threshold?    │
                   │ (0.3m)                 │
                   └─────┬────────────────┬─┘
                         │ NO             │ YES
                         │                ↓
                         │      ┌──────────────────┐
                         │      │ Advance waypoint │
                         │      │ idx++            │
                         │      └────────┬─────────┘
                         │               │
                         │      ┌────────▼──────────┐
                         │      │ At last waypoint? │
                         │      └──┬─────────────┬──┘
                         │         │ YES        │ NO
                         │         ↓            ↓
                         │   RETURN [0,0]  (get next)
                         │   (goal reached)
                         │                │
                         └────────────────┘
                                │
                                ↓
                   ┌────────────────────────────────┐
                   │ Calculate desired angle:       │
                   │ desired_angle =                │
                   │   atan2(dy, dx)                │
                   └────────────┬───────────────────┘
                                │
                                ↓
                   ┌────────────────────────────────┐
                   │ Calculate angle error:         │
                   │ error = desired - current      │
                   │ Normalize to [-π, π]           │
                   └────────────┬───────────────────┘
                                │
                                ↓
                   ┌────────────────────────────────┐
                   │ Proportional steering:         │
                   │ turn = K_p × error             │
                   │       K_p = 3.0 (gain)         │
                   └────────────┬───────────────────┘
                                │
                                ↓
                   ┌────────────────────────────────┐
                   │ Forward speed:                 │
                   │ forward = K_f × base ×         │
                   │           cos(error)           │
                   │ K_f = 0.8, base = 2.5         │
                   └────────────┬───────────────────┘
                                │
                                ↓
                   ┌────────────────────────────────┐
                   │ Convert to motor speeds:       │
                   │ left = forward + turn          │
                   │ right = forward - turn         │
                   └────────────┬───────────────────┘
                                │
                                ↓
                   ┌────────────────────────────────┐
                   │ Clamp to max velocity:         │
                   │ left = clamp(left, -20, +20)   │
                   │ right = clamp(right, -20, +20) │
                   └────────────┬───────────────────┘
                                │
                                ↓
                   ┌────────────────────────────────┐
                   │ RETURN [left, right]           │
                   └────────────────────────────────┘
```

---

## Behavior Selection Flowchart

```
┌─────────────────────────────────────┐
│ Every 32ms (control loop iteration) │
└────────────────┬────────────────────┘
                 │
                 ↓
    ┌────────────────────────────┐
    │ Step 1: Detect user?       │
    │ (RGB camera, HSV color)    │
    └──────┬──────────────────┬──┘
           │ YES              │ NO
           ↓                  ↓
    ┌──────────────────┐  ┌──────────────┐
    │ User following:  │  │ Continue...  │
    │ left, right =    │  └──────┬───────┘
    │ compute_follow   │         │
    │ _motor_speeds()  │         ↓
    │                  │  ┌───────────────────┐
    │ navigation_goal  │  │ Step 2: Path      │
    │ = None           │  │ planned & active? │
    │ (cancel path)    │  │ (len(path) > 0)   │
    │                  │  └──┬─────────────┬──┘
    └────────┬─────────┘     │ YES         │ NO
             │               ↓             ↓
             │       ┌──────────────────┐ ┌───────────┐
             │       │ Waypoint following:  │ Explore:  │
             │       │ left, right =     │ left,right=
             │       │ compute_waypoint  │ compute    │
             │       │ _motor_speeds()   │ _motor     │
             │       │                   │ _speeds()  │
             │       │                   │ (obstacle  │
             │       │                   │  avoidance)│
             │       └─────────┬─────────┘ └────┬──────┘
             │                 │                │
             │       ┌─────────▼────────────────▼┐
             │       │ Motor speeds calculated   │
             │       │ left, right (m/s)        │
             │       └──────────┬────────────────┘
             │                  │
             └──────────────────┘
                        │
                        ↓
        ┌───────────────────────────────┐
        │ APPLY TO MOTORS              │
        │ set_motor_velocities()       │
        │ ├─ front_left = left        │
        │ ├─ front_right = right      │
        │ ├─ rear_left = left         │
        │ └─ rear_right = right       │
        └───────────────┬─────────────┘
                        │
                        ↓
        ┌───────────────────────────────┐
        │ PHYSICS SIMULATION           │
        │ ├─ Calculate new position   │
        │ ├─ Update robot orientation │
        │ ├─ Rope physics (TurtleBot) │
        │ └─ Collision detection      │
        └───────────────┬─────────────┘
                        │
                        ↓
        ┌───────────────────────────────┐
        │ NEXT ITERATION               │
        │ Wait 32ms                    │
        │ Sensor data → line 1         │
        └───────────────────────────────┘
```

---

## Occupancy Grid Visualization

```
Before Path Planning (Step 1400):
┌─────────────────────────────────────────┐
│ Grid: 200×200 (5m × 5m area)           │
│ Resolution: 0.05m (5cm) per cell        │
│                                          │
│  0           100          200           │
│  ┌───┬───┬───┬───┬───┬───┬───┬───┬───┐ │
│0 │███│███│███│███│███│███│███│███│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│░░░│░░░│███│███│███│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│[R]│░░░│███│███│███│███│ │ Legend:
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │ ███ = Occupied (100)
│  │███│███│░░░│░░░│░░░│███│███│███│███│ │ ░░░ = Free (0)
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │ [R] = Robot
│  │███│███│░░░│░░░│░░░│███│███│███│███│ │ [G] = Goal
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│░░░│░░░│░░░│░░░│░░░│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│░░░│░░░│░░░│░░░│░░░│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│░░░│░░░│░░░│[G]│░░░│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│███│███│███│███│███│███│███│ │
│                                          │
│ [R] at grid (140, 130)                   │
│ [G] at grid (160, 145)                   │
│ Open cells form corridor: path possible  │
└─────────────────────────────────────────┘

After A* Planning (Step 1500):
┌─────────────────────────────────────────┐
│ A* search path highlighted:              │
│                                          │
│  0           100          200           │
│  ┌───┬───┬───┬───┬───┬───┬───┬───┬───┐ │
│0 │███│███│███│███│███│███│███│███│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│░░░│░░░│███│███│███│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│[●]│░░░│███│███│███│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│░●░│░░░│███│███│███│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│░●░│░░░│███│███│███│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│░●░│░●░│░●░│░●░│░░░│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│░●░│░░░│░░░│░●░│░░░│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│░░░│░░░│░░░│░░░│[●]│░░░│███│ │
│  ├───┼───┼───┼───┼───┼───┼───┼───┼───┤ │
│  │███│███│███│███│███│███│███│███│███│ │
│                                          │
│ [●] marks waypoints in path             │
│ Path: 38 waypoints total                │
│ Robot will follow from [R] to [G]       │
└─────────────────────────────────────────┘

Robot's View (Following Waypoint):
Current:   (140, 130)
Waypoint:  (141, 131)
Vector:    (1, 1)
Angle:     45°
Heading:   30°
Error:     +15°
→ Turn left slightly, move forward
```

---

## Timeline Diagram

```
SIMULATION TIMELINE (32ms timesteps)

Step 0:    Initialize
           SLAM: 100 particles
           Grid: all unknown (50)
           
Steps 1-100: Explore
            Robot moves randomly
            SLAM builds initial map
            Obstacles appear in grid
            
Steps 100-500: Improve Map
              Map becomes more detailed
              Particles converge to true pos
              Grid = 80% accuracy
              
Steps 500-1500: Maintain Map
               Continued exploration
               Fine-tuning accuracy
               Grid = 95% accurate
               No path planning yet
               
STEP 1500: ⭐ PATH PLANNING TRIGGERED
          Robot: (0.5m, -0.3m)
          Goal:  (2.5m, 0.2m)
          A* runs: ~50ms
          Result: 38 waypoints
          
Steps 1501-1600: Follow WP 1-10
                Distance to goal: 2.0m
                
Steps 1600-1700: Follow WP 11-20
                Distance to goal: 1.4m
                
Steps 1700-1800: Follow WP 21-30
                Distance to goal: 0.8m
                
Steps 1800-1900: Follow WP 31-37
                Distance to goal: 0.3m
                
Step 1900-2000: Final waypoint
               Distance to goal: < 0.1m
               ✓ Goal reached!
               Motor commands: [0, 0]
               
Steps 2000+: Waiting
            Ready for new goal
            Or user detection
            Or new command
```

---

## Control Flow With User Interruption

```
SCENARIO: User appears at Step 1750

Step 1750: Following waypoint #20
          Path: [●●●●●●●●●●●●●●●●●●●●●●●]
                                  ↑
                            current position
          
          Motor: left=2.1, right=1.9
          
          THEN: Person enters view!
          
Step 1751: behavior_selector():
          
          ┌─ User detected? YES!
          │
          ├─ Set: left_speed, right_speed = 
          │       compute_follow_motor_speeds()
          │
          ├─ Set: navigation_goal = None
          ├─ Set: current_path = []
          │
          └─ Motor: [switch to following]
          
Step 1752+: User following mode
           Following person with RGB-D
           Rope pulls TurtleBot3
           Path planning suspended
           
User moves away...
           
Step 1850: User no longer detected
          
          ┌─ User detected? NO
          │
          ├─ Path planned? NO (was cancelled)
          │
          └─ Use exploration mode
             (obstacle avoidance)
             
          Now waiting for:
          - New user to appear, OR
          - New navigation goal to be set
```

---

## Memory and Performance

```
MEMORY USAGE:

Occupancy Grid:
  200 × 200 cells = 40,000 cells
  Each cell: 1 byte (0-100 value)
  Total: 40 KB

Particles (SLAM):
  100 particles
  Per particle: x(float), y(float), theta(float) = 12 bytes
  Total: 1.2 KB

Waypoint Path:
  ~40 waypoints average
  Per waypoint: x(float), y(float) = 8 bytes
  Total: 320 bytes

Total: ~42 KB (very small, runs easily)

COMPUTATION TIME:

LIDAR Processing: ~5ms
  - 640 range values
  - Angle calculation
  - Grid updating
  
SLAM Update: ~10ms
  - Particle filter update
  - Resampling (when needed)
  - Pose estimation
  
A* Pathfinding: ~50ms (runs once per goal)
  - 40 waypoints = 40 iterations
  - Each iteration: O(log n) heap operations
  - Converts coordinates
  
Waypoint Following: <1ms
  - Math: distance, angle, motor speeds
  - Very fast
  
Total per 32ms timestep: <20ms (leaves margin)
Allows 30+ Hz control loop ✓
```

---

All diagrams created! 🎉
