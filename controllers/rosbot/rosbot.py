#!/opt/homebrew/bin/python3
"""
Rosbot controller with integrated SLAM (Particle Filter) and obstacle avoidance.
Derived from the previous standalone SLAM controller to avoid maintaining a
separate controller entry point.
"""

import math
import json
import random

try:
    from controller import (
        Robot,
        Motor,
        Lidar,
        Camera,
        RangeFinder,
        Accelerometer,
        Gyro,
        Compass,
        PositionSensor,
    )
    WEBOTS_AVAILABLE = True
except ImportError:
    print("Warning: Webots controller module not found.")
    WEBOTS_AVAILABLE = False
    Robot = None


class Particle:
    """Particle for Monte Carlo Localization."""

    def __init__(self, x, y, theta, weight=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

    def copy(self):
        return Particle(self.x, self.y, self.theta, self.weight)


class ParticleFilter:
    """Monte Carlo Localization (Particle Filter) for SLAM."""

    def __init__(self, num_particles=100, map_size=200, resolution=0.05):
        self.num_particles = num_particles
        self.particles = []
        self.map_size = map_size
        self.resolution = resolution
        self.occupancy_grid = [
            [-1 for _ in range(map_size)] for _ in range(map_size)
        ]  # -1 unknown, 0 free, 100 occupied
        self.map_center = [map_size // 2, map_size // 2]

        # Initialisation of the particles randomly
        self.initialize_particles()

        # Motion model noise
        self.motion_noise_linear = 0.05
        self.motion_noise_angular = 0.1

        # Sensor model parameters
        self.hit_prob = 0.9
        self.miss_prob = 0.1
        self.max_range = 12.0

    def initialize_particles(self):
        """Initialize particles with uniform distribution."""
        self.particles = []
        for _ in range(self.num_particles):
            x = random.gauss(0, 0.1)
            y = random.gauss(0, 0.1)
            theta = random.uniform(-math.pi, math.pi)
            self.particles.append(Particle(x, y, theta, 1.0 / self.num_particles))

    def predict(self, dx, dy, dtheta):
        """Motion model: predict particle positions based on odometry."""
        for particle in self.particles:
            particle.x += dx + random.gauss(0, self.motion_noise_linear)
            particle.y += dy + random.gauss(0, self.motion_noise_linear)
            particle.theta += dtheta + random.gauss(0, self.motion_noise_angular)

            while particle.theta > math.pi:
                particle.theta -= 2 * math.pi
            while particle.theta < -math.pi:
                particle.theta += 2 * math.pi

    def update(self, lidar_ranges, lidar_angles):
        """Measurement model: update particle weights based on lidar scan."""
        if not lidar_ranges or len(lidar_ranges) == 0:
            return

        total_weight = 0.0

        for particle in self.particles:
            weight = 1.0
            for range_val, angle in zip(lidar_ranges, lidar_angles):
                if math.isnan(range_val) or math.isinf(range_val) or range_val <= 0:
                    continue

                expected_range = self.raycast_map(
                    particle.x, particle.y, particle.theta + angle
                )

                if expected_range is not None:
                    error = abs(range_val - expected_range)
                    if error < 0.1:
                        weight *= self.hit_prob
                    else:
                        weight *= self.miss_prob * math.exp(-error / 0.5)
                else:
                    weight *= 0.5

            particle.weight = weight
            total_weight += weight

        if total_weight > 0:
            for particle in self.particles:
                particle.weight /= total_weight
        else:
            for particle in self.particles:
                particle.weight = 1.0 / self.num_particles

    def raycast_map(self, x, y, angle):
        """Raycast from position in direction to find expected range."""
        grid_x = int(self.map_center[0] + x / self.resolution)
        grid_y = int(self.map_center[1] + y / self.resolution)

        if not (0 <= grid_x < self.map_size and 0 <= grid_y < self.map_size):
            return None

        step_size = self.resolution
        max_steps = int(self.max_range / step_size)

        for step in range(max_steps):
            check_x = grid_x + int(step * math.cos(angle) / self.resolution)
            check_y = grid_y + int(step * math.sin(angle) / self.resolution)

            if not (0 <= check_x < self.map_size and 0 <= check_y < self.map_size):
                return step * step_size

            if self.occupancy_grid[check_y][check_x] == 100:
                return step * step_size

        return self.max_range

    def resample(self):
        """Resample particles based on weights."""
        new_particles = []
        weights = [p.weight for p in self.particles]

        step = 1.0 / self.num_particles
        u = random.uniform(0, step)
        c = weights[0]
        i = 0

        for _ in range(self.num_particles):
            while u > c and i < len(weights) - 1:
                i += 1
                c += weights[i]
            new_particles.append(self.particles[i].copy())
            u += step

        self.particles = new_particles

        num_random = int(self.num_particles * 0.1)
        for _ in range(num_random):
            x = random.gauss(0, 1.0)
            y = random.gauss(0, 1.0)
            theta = random.uniform(-math.pi, math.pi)
            self.particles[random.randint(0, len(self.particles) - 1)] = Particle(
                x, y, theta, 1.0 / self.num_particles
            )

    def get_best_estimate(self):
        """Get best pose estimate (weighted average of particles)."""
        if not self.particles:
            return 0.0, 0.0, 0.0

        total_weight = sum(p.weight for p in self.particles)
        if total_weight == 0:
            x = sum(p.x for p in self.particles) / len(self.particles)
            y = sum(p.y for p in self.particles) / len(self.particles)
            sin_sum = sum(math.sin(p.theta) for p in self.particles)
            cos_sum = sum(math.cos(p.theta) for p in self.particles)
            theta = math.atan2(sin_sum, cos_sum)
        else:
            x = sum(p.x * p.weight for p in self.particles) / total_weight
            y = sum(p.y * p.weight for p in self.particles) / total_weight
            sin_sum = sum(math.sin(p.theta) * p.weight for p in self.particles)
            cos_sum = sum(math.cos(p.theta) * p.weight for p in self.particles)
            theta = math.atan2(sin_sum, cos_sum)

        return x, y, theta

    def update_map(self, lidar_ranges, lidar_angles, robot_x, robot_y, robot_theta):
        """Update occupancy grid map from lidar scan."""
        for range_val, angle in zip(lidar_ranges, lidar_angles):
            if math.isnan(range_val) or math.isinf(range_val) or range_val <= 0:
                continue

            global_angle = robot_theta + angle
            end_x = robot_x + range_val * math.cos(global_angle)
            end_y = robot_y + range_val * math.sin(global_angle)

            grid_x = int(self.map_center[0] + end_x / self.resolution)
            grid_y = int(self.map_center[1] + end_y / self.resolution)

            if 0 <= grid_x < self.map_size and 0 <= grid_y < self.map_size:
                self.occupancy_grid[grid_y][grid_x] = 100

            steps = int(range_val / self.resolution)
            for step in range(steps):
                free_x = robot_x + (range_val * step / steps) * math.cos(global_angle)
                free_y = robot_y + (range_val * step / steps) * math.sin(global_angle)
                grid_x = int(self.map_center[0] + free_x / self.resolution)
                grid_y = int(self.map_center[1] + free_y / self.resolution)

                if 0 <= grid_x < self.map_size and 0 <= grid_y < self.map_size:
                    if self.occupancy_grid[grid_y][grid_x] == -1:
                        self.occupancy_grid[grid_y][grid_x] = 0

    def save_map(self, filename="slam_map.json"):
        """Save map and pose estimate to file."""
        x, y, theta = self.get_best_estimate()
        map_data = {
            "resolution": self.resolution,
            "map_size": self.map_size,
            "occupancy_grid": self.occupancy_grid,
            "estimated_pose": {"x": x, "y": y, "theta": theta},
            "num_particles": self.num_particles,
        }
        with open(filename, "w") as f:
            json.dump(map_data, f)
        print(f"Map and pose saved to {filename}")
        print(
            f"Estimated pose: x={x:.2f}m, y={y:.2f}m, θ={math.degrees(theta):.1f}°"
        )


import heapq


class PathPlanner:
    """A* pathfinding algorithm for navigation using occupancy grid."""
    
    def __init__(self, map_size=200, resolution=0.05):
        self.map_size = map_size
        self.resolution = resolution
        self.map_center = [map_size // 2, map_size // 2]
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices."""
        gx = int(self.map_center[0] + x / self.resolution)
        gy = int(self.map_center[1] + y / self.resolution)
        print(gx,gy)
        return (gx, gy)
    
    def grid_to_world(self, gx, gy):
        """Convert grid indices to world coordinates."""
        x = (gx - self.map_center[0]) * self.resolution
        y = (gy - self.map_center[1]) * self.resolution
        return (x, y)
    
    def heuristic(self, pos, goal):
        """Euclidean distance heuristic."""
        return math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)
    
    def get_neighbors(self, pos, occupancy_grid):
        """Get valid neighboring cells (8-directional movement)."""
        neighbors = []
        x, y = pos
        
        # 8-directional movement (including diagonals)
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            # Check bounds
            if 0 <= nx < self.map_size and 0 <= ny < self.map_size:
                # Check if cell is not occupied (occupancy < 75 = free space)
                if occupancy_grid[ny][nx] < 75:
                    # Cost: 1.4 for diagonal, 1 for straight
                    cost = 1.4 if dx != 0 and dy != 0 else 1.0
                    neighbors.append(((nx, ny), cost))
        
        return neighbors
    
    def plan_path(self, start_world, goal_world, occupancy_grid):
        """
        Find path from start to goal using A* algorithm.
        Returns list of (x, y) world coordinates, or empty list if no path found.
        """
        # Convert world coordinates (metres) to grid indices used by the occupancy map
        start_grid = self.world_to_grid(start_world[0], start_world[1])
        goal_grid = self.world_to_grid(goal_world[0], goal_world[1])
        
        # Sanity-check the start and goal positions before starting the search
        if not self._is_valid_cell(start_grid, occupancy_grid):
            print(f"Start position invalid: {start_grid}")
            return []
        
        if not self._is_valid_cell(goal_grid, occupancy_grid):
            print(f"Goal position invalid: {goal_grid}")
            return []
        
        # Initialize A* data structures
        open_set = []
        heapq.heappush(open_set, (0, start_grid))  # Priority queue of (f-score, cell)
        
        came_from = {}  # To reconstruct path later
        g_score = {start_grid: 0}  # Cost from start to cell
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}
        closed_set = set()  # Visited cells
        
        # Main loop: expand the cheapest node until we reach the goal
        while open_set:
            _, current = heapq.heappop(open_set)  # Node with lowest f-score
            
            # Goal test
            if current == goal_grid:
                # Reconstruct path from came_from
                path = self._reconstruct_path(came_from, current)
                return [self.grid_to_world(gx, gy) for gx, gy in path]
            
            closed_set.add(current)
            
            # Examine every neighbour of the current cell
            for neighbor, cost in self.get_neighbors(current, occupancy_grid):
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + cost
                
                # If this path to neighbour is better, record it
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # If we exit the loop, no route was found
        print(f"No path found from {start_grid} to {goal_grid}")
        return []
    
    def _is_valid_cell(self, grid_pos, occupancy_grid):
        """Check if a grid cell is valid and not occupied."""
        gx, gy = grid_pos
        if not (0 <= gx < self.map_size and 0 <= gy < self.map_size):
            return False
        # Cell is valid if occupancy < 75 (not heavily occupied)
        return occupancy_grid[gy][gx] < 75
    
    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from A* search."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]


class RosbotSlamController:
    def __init__(self):
        if not WEBOTS_AVAILABLE:
            print("Error: Webots not available")
            return

        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        self.slam = ParticleFilter(num_particles=100, map_size=200, resolution=0.05)
        self.path_planner = PathPlanner(map_size=200, resolution=0.05)

        self.init_devices()

        self.camera_rgb = self.robot.getDevice("camera rgb")
        self.camera_depth = self.robot.getDevice("camera depth")


        if self.camera_rgb:
            self.camera_rgb.enable(self.time_step)


        if self.camera_depth:
            self.camera_depth.enable(self.time_step)

        self.last_wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self.robot_position = [0.0, 0.0, 0.0]
        self.robot_orientation = 0.0
        self.last_time = self.robot.getTime()
        self.last_odom_pose = [0.0, 0.0, 0.0]
        self.label_supported = hasattr(self.robot, "setLabel")

        self.wheel_radius = 0.05
        self.wheel_base = 0.22

        self.base_speed = 2.5
        self.max_velocity = 20.0

        # Path planning and waypoint following
        self.current_path = []
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.3  # Distance to consider waypoint reached (meters)
        self.navigation_goal = None  # Target position for path planning


        print("Rosbot SLAM Controller initialised")
        print(f"Time step: {self.time_step}ms")
        print(f"Particle Filter: {self.slam.num_particles} particles")
        print("Mode: SLAM Localization and Mapping")

    def init_devices(self):
        """Initialize Webots devices."""
        self.front_left_motor = self.robot.getDevice("fl_wheel_joint")
        self.front_right_motor = self.robot.getDevice("fr_wheel_joint")
        self.rear_left_motor = self.robot.getDevice("rl_wheel_joint")
        self.rear_right_motor = self.robot.getDevice("rr_wheel_joint")

        for motor in [
            self.front_left_motor,
            self.front_right_motor,
            self.rear_left_motor,
            self.rear_right_motor,
        ]:
            if motor:
                motor.setPosition(float("inf"))
                motor.setVelocity(0.0)

        self.front_left_ps = self.robot.getDevice("front left wheel motor sensor")
        self.front_right_ps = self.robot.getDevice("front right wheel motor sensor")
        self.rear_left_ps = self.robot.getDevice("rear left wheel motor sensor")
        self.rear_right_ps = self.robot.getDevice("rear right wheel motor sensor")

        for ps in [
            self.front_left_ps,
            self.front_right_ps,
            self.rear_left_ps,
            self.rear_right_ps,
        ]:
            if ps:
                ps.enable(self.time_step)

        self.lidar = self.robot.getDevice("laser")
        if self.lidar:
            self.lidar.enable(self.time_step)
            self.lidar.enablePointCloud()
            self.lidar_width = self.lidar.getHorizontalResolution()
            self.lidar_max_range = self.lidar.getMaxRange()
            self.lidar_min_range = self.lidar.getMinRange()
            self.lidar_fov = self.lidar.getFov()
            print(
                f"Lidar enabled: {self.lidar_width} points, "
                f"FOV: {math.degrees(self.lidar_fov):.1f}°, "
                f"Range: {self.lidar_min_range:.2f}-{self.lidar_max_range:.2f}m"
            )

        self.accelerometer = self.robot.getDevice("imu accelerometer")
        self.gyro = self.robot.getDevice("imu gyro")
        self.compass = self.robot.getDevice("imu compass")

        if self.accelerometer:
            self.accelerometer.enable(self.time_step)
        if self.gyro:
            self.gyro.enable(self.time_step)
        if self.compass:
            self.compass.enable(self.time_step)

        self.distance_sensors = [
            self.robot.getDevice("fl_range"),
            self.robot.getDevice("rl_range"),
            self.robot.getDevice("fr_range"),
            self.robot.getDevice("rr_range"),
        ]
        for ds in self.distance_sensors:
            if ds:
                ds.enable(self.time_step)

        self.display = self.robot.getDevice("slam_display")

        if self.display:
            self.display_width = self.display.getWidth()
            self.display_height = self.display.getHeight()
            print("SLAM display enabled:", self.display_width, "x", self.display_height)
        else:
            print("No display found")


    def compute_odometry(self):
        """Compute odometry from wheel encoders."""
        fl_pos = self.front_left_ps.getValue() if self.front_left_ps else 0.0
        fr_pos = self.front_right_ps.getValue() if self.front_right_ps else 0.0
        rl_pos = self.rear_left_ps.getValue() if self.rear_left_ps else 0.0
        rr_pos = self.rear_right_ps.getValue() if self.rear_right_ps else 0.0

        current_time = self.robot.getTime()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = self.time_step / 1000.0

        fl_vel = (fl_pos - self.last_wheel_positions[0]) / dt
        fr_vel = (fr_pos - self.last_wheel_positions[1]) / dt
        rl_vel = (rl_pos - self.last_wheel_positions[2]) / dt
        rr_vel = (rr_pos - self.last_wheel_positions[3]) / dt

        left_vel = (fl_vel + rl_vel) / 2.0
        right_vel = (fr_vel + rr_vel) / 2.0

        compass_vals = self.compass.getValues() if self.compass else [0, 0, 1]
        yaw = math.atan2(compass_vals[0], compass_vals[1])

        linear_vel = self.wheel_radius * (left_vel + right_vel) / 2.0
        dx = linear_vel * math.cos(yaw) * dt
        dy = linear_vel * math.sin(yaw) * dt
        dtheta = self.wheel_radius * (right_vel - left_vel) / self.wheel_base * dt

        self.robot_position[0] += dx
        self.robot_position[1] += dy
        self.robot_orientation = yaw

        self.last_wheel_positions = [fl_pos, fr_pos, rl_pos, rr_pos]
        self.last_time = current_time

        return dx, dy, dtheta

    def update_overlay(self, step_count, odom_pose, slam_pose):
        """Draw a small on-screen overlay with odom / SLAM pose."""
        if not self.label_supported:
            return
        ox, oy, otheta = odom_pose
        sx, sy, stheta = slam_pose
        label = (
            f"Step {step_count}\n"
            f"Odom: {ox:.2f}, {oy:.2f}, {math.degrees(otheta):.1f}°\n"
            f"SLAM: {sx:.2f}, {sy:.2f}, {math.degrees(stheta):.1f}°"
        )
        self.robot.setLabel(label, 0.01, 0.01, 0.08, 0x00FF66, 0.2, "Arial")

    def debug_draw_path(self, path, occupancy_grid):
        """Draw occupancy grid, start, goal, and path on the SLAM display."""
        if not self.display:
            return
        
        # Clear display
        self.display.setColor(0x000000)
        self.display.fillRectangle(0, 0, self.display_width, self.display_height)
        
        # Helper to convert grid cell (gx, gy) to display pixel
        def cell_to_pixel(gx, gy):
            px = int(gx / self.slam.map_size * self.display_width)
            py = int(gy / self.slam.map_size * self.display_height)
            return px, py
        
        # Draw occupied cells (grey)
        self.display.setColor(0x555555)
        for y in range(self.slam.map_size):
            for x in range(self.slam.map_size):
                if occupancy_grid[y][x] >= 75:
                    px, py = cell_to_pixel(x, y)
                    if 0 <= px < self.display_width and 0 <= py < self.display_height:
                        self.display.drawPixel(px, py)
        
        # Draw the planned path (blue)
        if path:
            self.display.setColor(0x0000FF)
            for world_pos in path:
                gx, gy = self.path_planner.world_to_grid(world_pos[0], world_pos[1])
                px, py = cell_to_pixel(gx, gy)
                if 0 <= px < self.display_width and 0 <= py < self.display_height:
                    self.display.drawPixel(px, py)

    def process_lidar(self):
        """Process lidar data for SLAM."""
        if not self.lidar:
            return

        try:
            range_image = self.lidar.getRangeImage()
        except Exception:
            range_image = None

        if not range_image or len(range_image) == 0:
            return

        angles = []
        angle_step = self.lidar_fov / len(range_image)
        for i in range(len(range_image)):
            angle = -self.lidar_fov / 2.0 + i * angle_step
            angles.append(angle)

        slam_x, slam_y, slam_theta = self.slam.get_best_estimate()
        self.slam.update_map(range_image, angles, slam_x, slam_y, slam_theta)
        self.slam.update(range_image, angles)

    def detect_front_obstacle(self):
        """Detect obstacles directly in front of robot."""
        if not self.lidar:
            return None, None

        try:
            range_image = self.lidar.getRangeImage()
            if not range_image or len(range_image) == 0:
                return None, None

            angle_step = self.lidar_fov / len(range_image)

            front_start_idx = int(len(range_image) * 0.4)
            front_end_idx = int(len(range_image) * 0.55)

            min_dist = float("inf")
            min_angle = 0.0

            for i in range(front_start_idx, front_end_idx):
                dist = range_image[i]
                if not (math.isnan(dist) or math.isinf(dist) or dist <= 0):
                    if dist < min_dist:
                        min_dist = dist
                        angle = -self.lidar_fov / 2.0 + i * angle_step
                        min_angle = angle

            if min_dist < float("inf"):
                return min_dist, min_angle
        except Exception:
            pass

        return None, None

    def compute_motor_speeds(self):
        """Compute motor speeds with obstacle avoidance."""
        front_obstacle_dist, front_obstacle_angle = self.detect_front_obstacle()

        self.reverse_if_needed(front_obstacle_dist)

        if front_obstacle_dist is not None and front_obstacle_dist < 0.2:
            if front_obstacle_angle is not None:
                backup_speed = -2.5
                turn_speed = 2.0
                if front_obstacle_angle > 0:
                    return [
                        backup_speed + turn_speed,
                        backup_speed - turn_speed * 0.3,
                    ]
                return [
                    backup_speed - turn_speed * 0.3,
                    backup_speed + turn_speed,
                ]
            return [-2.5, -2.5]

        if front_obstacle_dist is not None and front_obstacle_dist < 0.35:
            if front_obstacle_angle is not None:
                turn_speed = 3.0
                if front_obstacle_angle > 0:
                    return [turn_speed, -turn_speed * 0.1]
                return [-turn_speed * 0.1, turn_speed]
            return [0.0, 0.0]

        base_speeds = [self.base_speed, self.base_speed]

        avoidance_speed = [0.0, 0.0]
        if front_obstacle_dist is not None and front_obstacle_angle is not None:
            avoidance_strength = 8.0
            if front_obstacle_dist < 0.4:
                factor = (0.4 - front_obstacle_dist) / 0.4
                if front_obstacle_angle > 0:
                    avoidance_speed[0] += factor * avoidance_strength
                    avoidance_speed[1] -= factor * avoidance_strength * 0.9
                else:
                    avoidance_speed[0] -= factor * avoidance_strength * 0.9
                    avoidance_speed[1] += factor * avoidance_strength

        if front_obstacle_dist is not None and front_obstacle_dist < 0.40:
            motor_speed = [
                base_speeds[0] * 0.1 + avoidance_speed[0] * 1.5,
                base_speeds[1] * 0.1 + avoidance_speed[1] * 1.5,
            ]
        else:
            motor_speed = [
                base_speeds[0] + avoidance_speed[0] * 0.6,
                base_speeds[1] + avoidance_speed[1] * 0.6,
            ]

        motor_speed[0] = max(-self.max_velocity, min(motor_speed[0], self.max_velocity))
        motor_speed[1] = max(-self.max_velocity, min(motor_speed[1], self.max_velocity))

        return motor_speed

    def set_motor_velocities(self, left_speed, right_speed):
        """Set motor velocities."""
        if self.front_left_motor:
            self.front_left_motor.setVelocity(left_speed)
        if self.front_right_motor:
            self.front_right_motor.setVelocity(right_speed)
        if self.rear_left_motor:
            self.rear_left_motor.setVelocity(left_speed)
        if self.rear_right_motor:
            self.rear_right_motor.setVelocity(right_speed)

    def set_navigation_goal(self, goal_x, goal_y):
        """Set a navigation goal and plan path to it."""
        self.navigation_goal = (goal_x, goal_y)
        self.current_path = []
        self.current_waypoint_idx = 0
        
        # Plan path using current occupancy grid
        start_pos = (self.robot_position[0], self.robot_position[1])
        print(f"\n=== Path Planning ===")
        print(f"Start: ({start_pos[0]:.2f}, {start_pos[1]:.2f})")
        print(f"Goal: ({goal_x:.2f}, {goal_y:.2f})")
        
        self.current_path = self.path_planner.plan_path(start_pos, (goal_x, goal_y), self.slam.occupancy_grid)
        
        if self.current_path:
            print(f"Path found with {len(self.current_path)} waypoints")
            for i, wp in enumerate(self.current_path[:5]):  # Print first 5 waypoints
                print(f"  WP{i}: ({wp[0]:.2f}, {wp[1]:.2f})")
            self.current_waypoint_idx = 0
        else:
            print("No path found!")
    
    def compute_waypoint_motor_speeds(self):
        """Compute motor speeds to follow planned waypoints."""
        if not self.current_path or self.current_waypoint_idx >= len(self.current_path):
            # No path or reached end
            return [0.0, 0.0]
        
        # Get current waypoint
        waypoint = self.current_path[self.current_waypoint_idx]
        robot_x, robot_y = self.robot_position[0], self.robot_position[1]
        
        # Distance to waypoint
        dx = waypoint[0] - robot_x
        dy = waypoint[1] - robot_y
        distance_to_waypoint = math.sqrt(dx**2 + dy**2)
        
        # If waypoint reached, move to next one
        if distance_to_waypoint < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.current_path):
                print("✓ Reached goal!")
                return [0.0, 0.0]
            waypoint = self.current_path[self.current_waypoint_idx]
            dx = waypoint[0] - robot_x
            dy = waypoint[1] - robot_y
            distance_to_waypoint = math.sqrt(dx**2 + dy**2)
        
        # Desired angle to waypoint
        desired_angle = math.atan2(dy, dx)
        
        # Current robot angle
        current_angle = self.robot_orientation
        
        # Calculate angle error (normalize to [-pi, pi])
        angle_error = desired_angle - current_angle
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Proportional control for steering and speed
        # Stronger turn if angle error is large
        turn_gain = 3.0
        turn_component = turn_gain * angle_error
        
        # Speed reduces when heading is wrong
        forward_gain = 0.8
        forward_component = forward_gain * self.base_speed * max(0, math.cos(angle_error))
        
        # Motor speeds: base_speed + steering adjustment
        left_speed = forward_component + turn_component
        right_speed = forward_component - turn_component
        
        # Clamp to max velocity
        left_speed = max(-self.max_velocity, min(left_speed, self.max_velocity))
        right_speed = max(-self.max_velocity, min(right_speed, self.max_velocity))
        
        return [left_speed, right_speed]

    # COLOR + DEPTH DETECTION

    @staticmethod
    def rgb_to_hsv(r, g, b):
        """Convert RGB to HSV (float H in [0,360), S,V in [0,1])."""
        rf = r / 255.0
        gf = g / 255.0
        bf = b / 255.0

        maxi = max(rf, gf, bf)
        mini = min(rf, gf, bf)
        delta = maxi - mini

        v = maxi
        s = 0 if maxi == 0 else delta / maxi

        if delta == 0:
            h = 0
        elif maxi == rf:
            h = (60 * ((gf - bf) / delta)) % 360
        elif maxi == gf:
            h = 60 * (((bf - rf) / delta) + 2)
        else:
            h = 60 * (((rf - gf) / delta) + 4)

        return h, s, v
    
    def find_green_bbox(self):
        """
        Scan RGB image and return bounding box around green user:
        (min_x, min_y, max_x, max_y, pixel_count) or None if not found.
        """
        cam = self.camera_rgb
        if cam is None:
            return None

        w = cam.getWidth()
        h = cam.getHeight()
        img = cam.getImage()
        if img is None:
            return None

        min_x, min_y = w, h
        max_x, max_y = 0, 0
        pixel_count = 0

        # Step by 2 for speed
        for y in range(0, h, 2):
            for x in range(0, w, 2):
                r = cam.imageGetRed(img, w, x, y)
                g = cam.imageGetGreen(img, w, x, y)
                b = cam.imageGetBlue(img, w, x, y)

                H, S, V = self.rgb_to_hsv(r, g, b)

                hsv_green = (50 < H < 150) and (S > 0.30) and (V > 0.25)
                rgb_green = (g > r + 40 and g > b + 40)

                if not (hsv_green or rgb_green):
                    continue

                if x < min_x: min_x = x
                if x > max_x: max_x = x
                if y < min_y: min_y = y
                if y > max_y: max_y = y
                pixel_count += 1

        if pixel_count < 100:
            return None

        return min_x, min_y, max_x, max_y, pixel_count
    
    def get_user_depth(self, cx, cy):
        """
        Sample depth at the bounding box centre.
        Returns distance in meters or None if invalid.
        """
        depth_cam = self.camera_depth
        rgb_cam = self.camera_rgb
        if depth_cam is None or rgb_cam is None:
            return None

        w = rgb_cam.getWidth()
        h = rgb_cam.getHeight()

        dw = depth_cam.getWidth()
        dh = depth_cam.getHeight()
        depth_img = depth_cam.getRangeImage()
        if depth_img is None:
            return None

        # Map RGB → depth coordinates
        dx = int(cx * dw / w)
        dy = int(cy * dh / h)

        # Clamp to valid range
        dx = max(0, min(dx, dw - 1))
        dy = max(0, min(dy, dh - 1))

        depth = depth_img[dy * dw + dx]

        if math.isinf(depth) or math.isnan(depth):
            return None

        # Approximate Astra blind zone as <0.05m
        if depth < 0.05:
            return 0.3  # treat as very close

        return depth
    
    def detect_user(self):
        """
        Detect green user and estimate distance.

        Returns:
            (cx, cy, distance) or (None, None, None) if no user.
        """
        bbox = self.find_green_bbox()
        if bbox is None:
            return None, None, None

        min_x, min_y, max_x, max_y, pixels = bbox
        cx = (min_x + max_x) // 2
        cy = (min_y + max_y) // 2

        dist = self.get_user_depth(cx, cy)
        return cx, cy, dist

    def compute_follow_speed(self, user_distance):
        """
        Depth-based forward speed.
        """
        if user_distance is None:
            return 0.0

        d_close = 0.25
        d_far = 3.0
        min_spd = 0.5
        max_spd = 3.0

        if user_distance < d_close:
            forward = max_spd
        elif user_distance > d_far:
            forward = 0.0
        else:
            ratio = 1.0 - ((user_distance - d_close) / (d_far - d_close))
            forward = min_spd + ratio * (max_spd - min_spd)

        return forward

    def compute_follow_motor_speeds(self, forward):
        """
        Lidar-based obstacle avoidance + user-following forward speed.
        """
        left = forward
        right = forward

        if forward <= 0 or not self.lidar:
            return left, right

        front_obstacle_dist, front_obstacle_angle = self.detect_front_obstacle()

        if front_obstacle_dist is None:
            return left, right
        
        left_wall, right_wall = self.detect_side_walls()

        # Corner escape condition
        if left_wall < 0.3 and right_wall < 0.3 and front_obstacle_dist < 0.4:
            print("Corner detected → FORCE TURN LEFT")
            return -1.5, 3.0   # Back left wheel / forward right wheel

        
        if front_obstacle_dist < 0.2:
            print(f"Obstacle {front_obstacle_dist:.2f}m → reversing")
            left = -2.5
            right = -2.5
            return left, right
        
        if front_obstacle_dist < 0.25:
            print("Obstacle VERY close → spin in place")
            left = 3
            right = -3
            return left, right
        
        # Close → turn away based on side
        if front_obstacle_dist < 0.3:
            if front_obstacle_angle > 0:
                print("Obstacle on RIGHT → turn LEFT")
                left = 0
                right = 2.5
            else:
                print("Obstacle on LEFT → turn RIGHT")
                left = 2.5
                right = 0
            return left, right

        # Clamp
        left = max(-self.max_velocity, min(left, self.max_velocity))
        right = max(-self.max_velocity, min(right, self.max_velocity))
        return left, right
    
    # DRAW SLAM MAP

    def draw_slam_map(self):
        if not self.display:
            return
        
        print("Drawing map…")

        grid = self.slam.occupancy_grid
        size = self.slam.map_size

        # Clear the display
        self.display.setColor(0xFFFFFF)  # white
        self.display.fillRectangle(0, 0, self.display_width, self.display_height)

        cell_w = self.display_width / size
        cell_h = self.display_height / size

        # Draw occupancy grid
        for y in range(size):
            for x in range(size):
                v = grid[y][x]
                if v == -1:
                    self.display.setColor(0xAAAAAA)  # unknown = gray
                elif v == 0:
                    self.display.setColor(0xFFFFFF)  # free = white
                else:
                    self.display.setColor(0x000000)  # occupied = black

                px = int(x * cell_w)
                py = int(y * cell_h)
                self.display.fillRectangle(px, py, int(cell_w), int(cell_h))

        # Draw robot position
        rx, ry, rtheta = self.slam.get_best_estimate()

        gx = int((rx / self.slam.resolution + self.slam.map_center[0]) * cell_w)
        gy = int((ry / self.slam.resolution + self.slam.map_center[1]) * cell_h)

        self.display.setColor(0xFF0000)  # robot = red
        self.display.fillOval(gx - 3, gy - 3, 6, 6)

        # Draw heading arrow
        arrow_length = 12
        hx = gx + arrow_length * math.cos(rtheta)
        hy = gy + arrow_length * math.sin(rtheta)

        self.display.setColor(0xFF0000)
        self.display.drawLine(gx, gy, int(hx), int(hy))

    
    # MAIN LOOP

    def run(self):
        """Main control loop."""
        step_count = 0
        resample_counter = 0

        print("=" * 50)
        print("Starting SLAM + Vision Controller with Particle Filter...")
        print(f"Lidar: {'Enabled' if self.lidar else 'Not found'}")
        if self.lidar:
            print(f"Lidar FOV: {math.degrees(self.lidar_fov):.1f}°")
            print(f"Lidar resolution: {self.lidar_width} points")
        print(f"Particles: {self.slam.num_particles}")
        print("=" * 50)

        while self.robot.step(self.time_step) != -1:
            # 1) Decide behaviour: follow user if visible, else pure SLAM-avoidance
            cx, cy, user_distance = (None, None, None)
            if self.camera_rgb and self.camera_depth:
                cx, cy, user_distance = self.detect_user()

            if cx is not None:
                # User detected - follow user instead of planned path
                print(f"User detected at ({cx},{cy}), depth={user_distance}")
                forward = self.compute_follow_speed(user_distance)
                left_speed, right_speed = self.compute_follow_motor_speeds(forward)
                self.navigation_goal = None  # Cancel any active path planning
                self.current_path = []
            elif self.navigation_goal is not None and len(self.current_path) > 0:
                # Following planned path (no user detected)
                left_speed, right_speed = self.compute_waypoint_motor_speeds()
            else:
                # No user and no active path - pure obstacle avoidance
                left_speed, right_speed = self.compute_motor_speeds()

            self.set_motor_velocities(left_speed, right_speed)

            # 2) SLAM update (odometry + lidar)
            dx, dy, dtheta = self.compute_odometry()
            self.slam.predict(dx, dy, dtheta)

            if step_count % 3 == 0:      # update map ~10 times/sec
                self.draw_slam_map()

            if step_count % 5 == 0:
                self.process_lidar()
                resample_counter += 1
                if resample_counter >= 10:
                    self.slam.resample()
                    resample_counter = 0

            # 3) Debug + overlay
            if step_count % 100 == 0:
                slam_x, slam_y, slam_theta = self.slam.get_best_estimate()
                odom_x, odom_y = self.robot_position[0], self.robot_position[1]
                odom_theta = self.robot_orientation

                front_obstacle_dist, front_obstacle_angle = self.detect_front_obstacle()
                if front_obstacle_dist is not None:
                    obstacle_info = (
                        f", Front: {front_obstacle_dist:.2f}m "
                        f"@ {math.degrees(front_obstacle_angle):.1f}°"
                    )
                else:
                    obstacle_info = ", Front: CLEAR"

                print(f"Step {step_count}:")
                print(
                    f"  Odometry: x={odom_x:.2f}m, y={odom_y:.2f}m, "
                    f"θ={math.degrees(odom_theta):.1f}°"
                )
                print(
                    f"  SLAM Est: x={slam_x:.2f}m, y={slam_y:.2f}m, "
                    f"θ={math.degrees(slam_theta):.1f}°"
                )
                print(
                    f"  Motors: L={left_speed:.2f} R={right_speed:.2f}"
                    f"{obstacle_info}"
                )

            if step_count % 5 == 0:
                slam_x, slam_y, slam_theta = self.slam.get_best_estimate()
                self.update_overlay(
                    step_count,
                    (self.robot_position[0], self.robot_position[1], self.robot_orientation),
                    (slam_x, slam_y, slam_theta),
                )

            if step_count % 1000 == 0 and step_count > 0:
                self.slam.save_map(f"slam_map_{step_count}.json")
                print(f"Map and pose saved at step {step_count}")

            # Trigger path planning demo at step 1500 (let SLAM build map first)
            if step_count == 1500 and self.navigation_goal is None:
                # Set a goal location (2 meters to the right)
                goal_x = self.robot_position[0] + 2.0
                goal_y = self.robot_position[1] + 0.5
                print(f"\n*** TRIGGERING PATH PLANNING TEST ***")
                print(f"Current robot position: ({self.robot_position[0]:.2f}, {self.robot_position[1]:.2f})")
                self.set_navigation_goal(goal_x, goal_y)
            
            step_count += 1

        self.slam.save_map("slam_map_final.json")
        print("SLAM mapping complete!")


def visualize_apartment_grid(occupancy_grid, start=None, goal=None, path=None, map_size=200):
    """Display ASCII visualization of the apartment layout"""
    planner = PathPlanner(map_size=map_size, resolution=0.05)
    
    # Convert world coords to grid coords if provided
    start_grid = None
    goal_grid = None
    if start:
        start_grid = (planner.world_to_grid(start[0], start[1]))
    if goal:
        goal_grid = (planner.world_to_grid(goal[0], goal[1]))
    
    # Convert path to grid coords
    path_grid = []
    if path:
        for x, y in path:
            px, py = planner.world_to_grid(x, y)
            path_grid.append((px, py))
    
    # Scale down for display (show every Nth cell)
    scale = 4  # Show 1 cell per 4x4
    display_width = map_size // scale
    display_height = map_size // scale
    
    print("\n" + "="*70)
    print("APARTMENT LAYOUT VISUALIZATION")
    print("="*70)
    print(f"Legend: █=Wall  ░=Free  ░=Unknown  S=Start  G=Goal  *=Path")
    print()
    
    for y in range(0, map_size, scale):
        for x in range(0, map_size, scale):
            cell_value = occupancy_grid[y][x]
            
            # Check if this cell is part of the path
            is_path = False
            if path_grid:
                for px, py in path_grid:
                    if px // scale == x // scale and py // scale == y // scale:
                        is_path = True
                        break
            
            # Check if start or goal
            is_start = start_grid and start_grid[0] // scale == x // scale and start_grid[1] // scale == y // scale
            is_goal = goal_grid and goal_grid[0] // scale == x // scale and goal_grid[1] // scale == y // scale
            
            if is_start:
                print("S", end="")
            elif is_goal:
                print("G", end="")
            elif is_path:
                print("*", end="")
            elif cell_value >= 75:
                print("█", end="")
            elif cell_value >= 0:
                print("·", end="")
            else:
                print("?", end="")
        print()
    
    print()
    print("="*70 + "\n")


def create_realistic_apartment_grid():
    """Create apartment layout matching Webots domestic-environment.wbt"""
    size = 200
    grid = [[0 for _ in range(size)] for _ in range(size)]  # Start with all free space
    
    # Helper function to draw walls
    def draw_wall_h(x1, x2, y, thickness=1):
        """Draw horizontal wall"""
        for x in range(min(x1, x2), max(x1, x2) + 1):
            for t in range(thickness):
                if 0 <= x < size and 0 <= y + t < size:
                    grid[y + t][x] = 100
    
    def draw_wall_v(x, y1, y2, thickness=1):
        """Draw vertical wall"""
        for y in range(min(y1, y2), max(y1, y2) + 1):
            for t in range(thickness):
                if 0 <= x + t < size and 0 <= y < size:
                    grid[y][x + t] = 100
    
    # OUTER PERIMETER WALLS (arena walls)
    draw_wall_h(20, 180, 20, 2)        # Top boundary
    draw_wall_h(20, 180, 178, 2)       # Bottom boundary
    draw_wall_v(20, 20, 178, 2)        # Left boundary
    draw_wall_v(178, 20, 178, 2)       # Right boundary
    
    # MAIN CORRIDOR (horizontal, center of building)
    draw_wall_h(20, 180, 90, 1)        # Corridor top
    draw_wall_h(20, 180, 110, 1)       # Corridor bottom
    # Corridors are open space, not blocked
    
    # BEDROOM 1 (left side, above corridor)
    draw_wall_h(20, 80, 45, 1)         # Top wall
    draw_wall_v(80, 45, 90, 1)         # Right wall (doorway gap below)
    # Doorway from bedroom 1 to corridor at x=70-80, y=90
    for x in range(70, 80):
        grid[90][x] = 0
    
    # BEDROOM 2 (right side, above corridor)
    draw_wall_h(120, 180, 45, 1)       # Top wall
    draw_wall_v(120, 45, 90, 1)        # Left wall (doorway gap below)
    # Doorway from bedroom 2 to corridor at x=120-130, y=90
    for x in range(120, 130):
        grid[90][x] = 0
    
    # KITCHEN (left side, below corridor)
    draw_wall_h(20, 80, 115, 1)        # Bottom wall
    draw_wall_v(80, 110, 115, 1)       # Right wall (doorway gap above)
    # Doorway from kitchen to corridor at x=70-80, y=110
    for x in range(70, 80):
        grid[110][x] = 0
    
    # LIVING ROOM (right side, below corridor)
    draw_wall_h(120, 180, 115, 1)      # Bottom wall
    draw_wall_v(120, 110, 115, 1)      # Left wall (doorway gap above)
    # Doorway from living room to corridor at x=120-130, y=110
    for x in range(120, 130):
        grid[110][x] = 0
    
    # MIDDLE OBSTACLES/WALLS (add some interior walls like in Webots)
    # Toilet area
    draw_wall_v(100, 65, 75, 1)        # Vertical wall in middle
    draw_wall_h(95, 105, 70, 1)        # Horizontal wall
    
    return grid, size


def test_path_planner_real_apartment():
    """Test path planning against realistic apartment layout"""
    import time
    
    # Try to load real SLAM map
    try:
        with open("slam_map_final.json", "r") as f:
            data = json.load(f)
        print("✓ Loaded REAL apartment map from slam_map_final.json")
        occupancy_grid = data["occupancy_grid"]
        map_size = data["map_size"]
        is_real = True
    except FileNotFoundError:
        print("⚠ No Webots map found. Creating realistic synthetic apartment...")
        occupancy_grid, map_size = create_realistic_apartment_grid()
        is_real = False
    
    planner = PathPlanner(map_size=map_size, resolution=0.05)
    
    # Analyze occupancy grid
    occupied = sum(1 for row in occupancy_grid for cell in row if cell >= 75)
    free = sum(1 for row in occupancy_grid for cell in row if cell < 75)
    unknown = sum(1 for row in occupancy_grid for cell in row if cell == -1)
    total = map_size * map_size
    
    print("\n" + "="*70)
    print("APARTMENT OCCUPANCY ANALYSIS")
    print("="*70)
    print(f"Total cells: {total}")
    print(f"  Occupied (≥75%): {occupied} ({100*occupied/total:.1f}%)")
    print(f"  Free (<75%):     {free} ({100*free/total:.1f}%)")
    print(f"  Unknown (-1):    {unknown} ({100*unknown/total:.1f}%)")
    print()
    
    # Show apartment layout
    print("Overall apartment layout (every 4th cell):")
    visualize_apartment_grid(occupancy_grid, map_size=map_size)
    
    planner = PathPlanner(map_size=map_size, resolution=0.05)
    
    # Test cases across the apartment
    test_cases = [
        # Simple paths within hallway
        ((0.0, 0.0), (0.25, 0.25), "Short diagonal in hallway"),
        ((0.0, 0.0), (0.5, 0.0), "Short horizontal in hallway"),
        ((0.0, 0.0), (0.0, 0.5), "Short vertical in hallway"),
        
        # Hallway traversals
        ((0.0, -2.0), (0.0, 2.0), "Full hallway north-south"),
        ((-1.5, 0.0), (1.5, 0.0), "Hallway east-west"),
        
        # Into bedrooms
        ((0.0, 0.0), (-1.5, -1.5), "Into Bedroom 1"),
        ((0.0, 0.0), (1.5, -1.5), "Into Bedroom 2"),
        
        # Into kitchen/living
        ((0.0, 0.0), (-1.5, 1.5), "Into Kitchen"),
        ((0.0, 0.0), (1.5, 1.5), "Into Living room"),
        
        # Cross-apartment paths
        ((-1.5, -1.5), (1.5, 1.5), "Corner to corner"),
        ((-2.0, -2.0), (2.0, -2.0), "Top left to top right"),
        ((-2.0, 2.0), (2.0, 2.0), "Bottom left to bottom right"),
    ]
    
    print("="*70)
    print("PATH PLANNING RESULTS" + (" [REAL APARTMENT]" if is_real else " [MOCK GRID]"))
    print("="*70)
    
    successful = 0
    failed = 0
    total_waypoints = 0
    total_time = 0
    
    for start, goal, desc in test_cases:
        print(f"\nTest: {desc}")
        print(f"  Start: ({start[0]:.1f}, {start[1]:.1f}) → Goal: ({goal[0]:.1f}, {goal[1]:.1f})")
        
        try:
            start_time = time.time()
            path = planner.plan_path(start, goal, occupancy_grid)
            elapsed = time.time() - start_time
            total_time += elapsed
            
            if path:
                successful += 1
                total_waypoints += len(path)
                
                # Calculate path length
                path_length = 0
                for i in range(len(path) - 1):
                    dx = path[i+1][0] - path[i][0]
                    dy = path[i+1][1] - path[i][1]
                    path_length += (dx**2 + dy**2)**0.5
                
                euclidean = ((goal[0]-start[0])**2 + (goal[1]-start[1])**2)**0.5
                efficiency = euclidean / path_length if path_length > 0 else 0
                
                print(f"  ✓ Path found!")
                print(f"    Waypoints: {len(path)}")
                print(f"    Path length: {path_length:.2f}m (Euclidean: {euclidean:.2f}m)")
                print(f"    Efficiency: {efficiency*100:.1f}% of straight line")
                print(f"    Time: {elapsed*1000:.1f}ms")
                
                # Show path visualization
                visualize_apartment_grid(occupancy_grid, start=start, goal=goal, path=path, map_size=map_size)
            else:
                failed += 1
                print(f"  ✗ No path found")
                print(f"    Time: {elapsed*1000:.1f}ms")
        except Exception as e:
            failed += 1
            print(f"  ✗ Error: {e}")
    
    print("="*70)
    print("SUMMARY")
    print("="*70)
    print(f"Tests run: {len(test_cases)}")
    print(f"Successful: {successful} ({100*successful/len(test_cases):.0f}%)")
    print(f"Failed: {failed} ({100*failed/len(test_cases):.0f}%)")
    if successful > 0:
        print(f"Average waypoints: {total_waypoints/successful:.1f}")
        print(f"Average planning time: {total_time/len(test_cases)*1000:.1f}ms")
    print()
    
    if is_real:
        print("✓ Using REAL Webots apartment map")
    else:
        print("📌 Using SYNTHETIC apartment layout:")
        print("   - 2 Bedrooms with doorways")
        print("   - Kitchen/Dining")
        print("   - Living room")
        print("   - Central hallway")
        print("   - Small bathroom")
        print()
        print("   To use REAL Webots map instead:")
        print("   1. Run: webots worlds/domestic-environment.wbt")
        print("   2. Let SLAM explore and complete")
        print("   3. The map saves as slam_map_final.json")
        print("   4. Run this test again to load it automatically")
    
    print("="*70 + "\n")


def test_path_planner_standalone():
    """Test PathPlanner without Webots or full SLAM."""
    import json
    
    # Try to load a saved occupancy grid, or create a mock one
    def load_or_create_grid():
        try:
            with open("slam_map_final.json", "r") as f:
                data = json.load(f)
            print("✓ Loaded occupancy grid from slam_map_final.json")
            return data["occupancy_grid"], data["map_size"]
        except FileNotFoundError:
            print("No saved map found. Creating mock grid...")
            size = 200
            grid = [[-1 for _ in range(size)] for _ in range(size)]
            # Free space in center
            for y in range(50, 150):
                for x in range(50, 150):
                    grid[y][x] = 0
            # Add vertical walls
            for y in range(80, 120):
                for x in range(80, 85):
                    grid[y][x] = 100
                for x in range(115, 120):
                    grid[y][x] = 100
            return grid, size
    
    occupancy_grid, map_size = load_or_create_grid()
    planner = PathPlanner(map_size=map_size, resolution=0.05)
    
    # Test cases
    test_cases = [
        ((0.0, 0.0), (2.0, 1.0), "Right and forward"),
        ((0.0, 0.0), (-1.5, -1.5), "Left and backward"),
        ((0.0, 0.0), (0.0, 0.0), "Start equals goal"),
    ]
    
    print("\n" + "="*60)
    print("PATH PLANNER TEST")
    print("="*60)
    
    for start, goal, desc in test_cases:
        print(f"\nTest: {desc}")
        print(f"  Start: {start}, Goal: {goal}")
        path = planner.plan_path(start, goal, occupancy_grid)
        if path:
            print(f"  ✓ Path found: {len(path)} waypoints")
            for i, (x, y) in enumerate(path):
                print(f"    WP{i}: ({x:.2f}, {y:.2f})")
        else:
            print(f"  ✗ No path found")


def main():
    controller = RosbotSlamController()
    if controller.robot:
        controller.run()


if __name__ == "__main__":
    # Choose which test to run:
    
    # Test 1: Simple mock grid with 2 walls
    # test_path_planner_standalone()
    
    # Test 2: Realistic synthetic apartment layout (NO WEBOTS NEEDED!)
    test_path_planner_real_apartment()
    
    # Full controller with Webots simulation:
    # main()