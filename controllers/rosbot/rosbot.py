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
    # Particle class for the Monte Carlo Localisation, where it initialises the particles and updates the weights.

    def __init__(self, x, y, theta, weight=1.0): 
        self.x = x 
        self.y = y 
        self.theta = theta 
        self.weight = weight

    # Then it copies the particle to a new particle.
    def copy(self): 
        return Particle(self.x, self.y, self.theta, self.weight)



class ParticleFilter:
    # The Particle Filter is the main class for the Monte Carlo Localisation, where it initialises the particles and updates the weights.

    def __init__(self, num_particles=100, map_size=200, resolution=0.05):
        self.num_particles = num_particles
        self.particles = []
        self.map_size = map_size
        self.resolution = resolution
        # Log-odds grid for occupancy mapping (0 = unknown / 0.5 prob)
        self.log_odds_grid = [
            [0.0 for _ in range(map_size)] for _ in range(map_size)
        ]
        # Convenience probability grid for visualization / raycast (0–100 scale)
        self.occupancy_grid = [
            [50 for _ in range(map_size)] for _ in range(map_size)
        ]  # 50 => p=0.5 unknown, 0 free, 100 occupied
        self.map_center = [map_size // 2, map_size // 2]

        # Initialises the particles randomly.
        self.initialise_particles()

        # Motion model noise, where it's the noise of the linear and angular motion.
        self.motion_noise_linear = 0.05
        self.motion_noise_angular = 0.1

        # Sensor model parameters
        self.hit_prob = 0.9
        self.free_prob = 0.35
        self.miss_prob = 0.1
        self.prior_prob = 0.5
        self.max_range = 12.0

    def initialise_particles(self):
        # Initialises the particles randomly. The gaussian distribution is used to generate the particles and the uniform distribution is used to generate the theta.
        self.particles = []
        for _ in range(self.num_particles):
            x = random.gauss(0, 0.1)
            y = random.gauss(0, 0.1)
            theta = random.uniform(-math.pi, math.pi)
            self.particles.append(Particle(x, y, theta, 1.0 / self.num_particles))

    def predict(self, dx, dy, dtheta):
        # As it predicts the particle positions based on the odometry, it adds the noise to the particle positions so that it can predict the particle positions a bit more accurately.
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
        def logit(p):
            return math.log(p / (1.0 - p))

        def logistic(l):
            return 1.0 / (1.0 + math.exp(-l))

        hit_inc = logit(self.hit_prob)
        free_inc = logit(self.free_prob)
        min_lo, max_lo = -10.0, 10.0  # avoid overflow / saturation

        for range_val, angle in zip(lidar_ranges, lidar_angles):
            if math.isnan(range_val) or math.isinf(range_val) or range_val <= 0:
                continue

            global_angle = robot_theta + angle
            end_x = robot_x + range_val * math.cos(global_angle)
            end_y = robot_y + range_val * math.sin(global_angle)

            hit_x = int(self.map_center[0] + end_x / self.resolution)
            hit_y = int(self.map_center[1] + end_y / self.resolution)

            steps = max(1, int(range_val / self.resolution))
            for step in range(steps):
                free_x = robot_x + (range_val * step / steps) * math.cos(global_angle)
                free_y = robot_y + (range_val * step / steps) * math.sin(global_angle)
                grid_x = int(self.map_center[0] + free_x / self.resolution)
                grid_y = int(self.map_center[1] + free_y / self.resolution)

                if 0 <= grid_x < self.map_size and 0 <= grid_y < self.map_size:
                    lo = self.log_odds_grid[grid_y][grid_x] + free_inc
                    lo = max(min(lo, max_lo), min_lo)
                    self.log_odds_grid[grid_y][grid_x] = lo
                    p = logistic(lo)
                    self.occupancy_grid[grid_y][grid_x] = int(p * 100)

            if 0 <= hit_x < self.map_size and 0 <= hit_y < self.map_size:
                lo = self.log_odds_grid[hit_y][hit_x] + hit_inc
                lo = max(min(lo, max_lo), min_lo)
                self.log_odds_grid[hit_y][hit_x] = lo
                p = logistic(lo)
                self.occupancy_grid[hit_y][hit_x] = int(p * 100)

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


class RosbotSlamController:
    def __init__(self):
        if not WEBOTS_AVAILABLE:
            print("Error: Webots not available")
            return

        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        self.slam = ParticleFilter(num_particles=100, map_size=200, resolution=0.05)

        self.init_devices()

        # Initialise wheel encoder baselines to avoid a huge first odometry jump
        def safe_val(sensor):
            try:
                val = sensor.getValue() if sensor else 0.0
                return val if math.isfinite(val) else 0.0
            except Exception:
                return 0.0

        fl0 = safe_val(self.front_left_ps)
        fr0 = safe_val(self.front_right_ps)
        rl0 = safe_val(self.rear_left_ps)
        rr0 = safe_val(self.rear_right_ps)
        self.last_wheel_positions = [fl0, fr0, rl0, rr0]
        self.robot_position = [0.0, 0.0, 0.0]
        # Seed heading from compass if available to align odometry yaw with world
        compass_vals = self.compass.getValues() if self.compass else [0, 0, 1]
        heading = math.atan2(compass_vals[0], compass_vals[1])
        self.robot_orientation = heading if math.isfinite(heading) else 0.0
        self.last_time = self.robot.getTime()
        self.last_odom_pose = [0.0, 0.0, 0.0]
        self.label_supported = hasattr(self.robot, "setLabel")

        self.wheel_radius = 0.05
        self.wheel_base = 0.22

        self.base_speed = 2.5
        self.max_velocity = 20.0

        print("Rosbot SLAM Controller initialised")
        print(f"Time step: {self.time_step}ms")
        print(f"Particle Filter: {self.slam.num_particles} particles")
        print("Mode: SLAM Localization and Mapping")

    def init_devices(self):
        """initialise Webots devices."""
        self.front_left_motor = self.robot.getDevice("fl_wheel_joint")
        self.front_right_motor = self.robot.getDevice("fr_wheel_joint")
        self.rear_left_motor = self.robot.getDevice("rl_wheel_joint")
        self.rear_right_motor = self.robot.getDevice("rr_wheel_joint")

        self.camera_rgb = self.robot.getDevice("camera rgb")
        # The depth camera from Astra is a RangeFinder device, not a Camera
        self.range_finder = self.robot.getDevice("camera depth")
        # Keep camera_depth for backward compatibility in get_user_depth
        self.camera_depth = self.range_finder

        if self.camera_rgb:
            self.camera_rgb.enable(self.time_step)

        if self.range_finder:
            self.range_finder.enable(self.time_step)
            # Verify the device supports RangeFinder methods
            if hasattr(self.range_finder, 'getRangeImage'):
                print(f"Range finder enabled: {self.range_finder.getWidth()}x{self.range_finder.getHeight()}, "
                      f"Range: {self.range_finder.getMinRange():.2f}-{self.range_finder.getMaxRange():.2f}m")
            else:
                print("Warning: Device 'camera depth' does not support getRangeImage() - may not be a RangeFinder")
                self.range_finder = None
        else:
            print("Warning: Range finder 'camera depth' not found")
            # Try alternative names
            alt_names = ["camera_depth", "depth", "range_finder"]
            for alt_name in alt_names:
                alt_device = self.robot.getDevice(alt_name)
                if alt_device and hasattr(alt_device, 'getRangeImage'):
                    self.range_finder = alt_device
                    self.range_finder.enable(self.time_step)
                    self.camera_depth = self.range_finder
                    print(f"Found range finder with alternative name: '{alt_name}'")
                    print(f"Range finder enabled: {self.range_finder.getWidth()}x{self.range_finder.getHeight()}, "
                          f"Range: {self.range_finder.getMinRange():.2f}-{self.range_finder.getMaxRange():.2f}m")
                    break

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
        """Compute differential-drive odometry from wheel encoders."""
        def finite_or_default(val, default=0.0):
            return val if math.isfinite(val) else default

        def safe_get(sensor):
            try:
                return finite_or_default(sensor.getValue()) if sensor else 0.0
            except Exception:
                return 0.0

        fl_pos = safe_get(self.front_left_ps)
        fr_pos = safe_get(self.front_right_ps)
        rl_pos = safe_get(self.rear_left_ps)
        rr_pos = safe_get(self.rear_right_ps)

        # If any encoder is still uninitialized (NaN), skip odometry update this step
        if not all(math.isfinite(p) for p in [fl_pos, fr_pos, rl_pos, rr_pos]):
            # refresh baselines so the next valid read uses these as zero
            self.last_wheel_positions = [fl_pos, fr_pos, rl_pos, rr_pos]
            return 0.0, 0.0, 0.0

        # Wheel travel since last step (radians); average front/rear per side
        delta_fl = fl_pos - self.last_wheel_positions[0]
        delta_fr = fr_pos - self.last_wheel_positions[1]
        delta_rl = rl_pos - self.last_wheel_positions[2]
        delta_rr = rr_pos - self.last_wheel_positions[3]

        delta_s_left = self.wheel_radius * (delta_fl + delta_rl) / 2.0
        delta_s_right = self.wheel_radius * (delta_fr + delta_rr) / 2.0

        # Reject implausible jumps from encoders (e.g., stale or wrapped values)
        max_step = (
            self.wheel_radius
            * self.max_velocity
            * (self.time_step / 1000.0)
            * 3.0  # small safety factor
        )
        if any(abs(v) > max_step for v in [delta_s_left, delta_s_right]):
            # Skip update this tick; keep last wheel positions to avoid compounding error
            self.last_wheel_positions = [fl_pos, fr_pos, rl_pos, rr_pos]
            return 0.0, 0.0, 0.0

        delta_s = (delta_s_right + delta_s_left) / 2.0
        delta_theta = (delta_s_right - delta_s_left) / self.wheel_base

        mid_theta = self.robot_orientation + delta_theta / 2.0
        delta_x = delta_s * math.cos(mid_theta)
        delta_y = delta_s * math.sin(mid_theta)

        self.robot_position[0] += delta_x
        self.robot_position[1] += delta_y
        self.robot_orientation = finite_or_default(self.robot_orientation + delta_theta, 0.0)

        # Normalize orientation to [-pi, pi] for stability
        self.robot_orientation = (self.robot_orientation + math.pi) % (
            2 * math.pi
        ) - math.pi

        self.last_wheel_positions = [fl_pos, fr_pos, rl_pos, rr_pos]
        self.last_time = self.robot.getTime()

        return delta_x, delta_y, delta_theta

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
        if not all(math.isfinite(v) for v in [slam_x, slam_y, slam_theta]):
            return

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

            front_start_idx = int(len(range_image) * 0.375)
            front_end_idx = int(len(range_image) * 0.625)

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
    
    def find_best_exploration_direction(self, obstacle_threshold=0.5):
        """
        Scan multiple directions and find the best direction to explore.
        Returns (best_angle, max_clear_distance) where best_angle is relative to robot heading.
        """
        if not self.lidar:
            return None, None

        try:
            range_image = self.lidar.getRangeImage()
            if not range_image or len(range_image) == 0:
                return None, None

            angle_step = self.lidar_fov / len(range_image)
            
            # Define directions to check (relative to robot front = 0)
            # Format: (name, center_angle_degrees, angular_width_degrees)
            directions_to_check = [
                ("front", 0, 45),           # -22.5 to +22.5 degrees
                ("front-left", -30, 30),    # -45 to -15 degrees
                ("left", -60, 30),          # -75 to -45 degrees
                ("front-right", 30, 30),    # +15 to +45 degrees
                ("right", 60, 30),          # +45 to +75 degrees
                ("left-back", -120, 30),    # -135 to -105 degrees
                ("right-back", 120, 30),    # +105 to +135 degrees
            ]
            
            best_angle = 0.0
            max_clear_distance = 0.0
            
            for direction_name, center_deg, width_deg in directions_to_check:
                # Convert to radians
                center_rad = math.radians(center_deg)
                half_width_rad = math.radians(width_deg / 2.0)
                
                # Find lidar indices corresponding to this direction
                # Lidar angles range from -fov/2 to +fov/2
                start_angle = center_rad - half_width_rad
                end_angle = center_rad + half_width_rad
                
                # Find the minimum distance in this direction (closest obstacle)
                min_dist_in_direction = float("inf")
                valid_distances = []
                
                for i, dist in enumerate(range_image):
                    if math.isnan(dist) or math.isinf(dist) or dist <= 0:
                        continue
                    
                    # Calculate angle of this lidar beam
                    lidar_angle = -self.lidar_fov / 2.0 + i * angle_step
                    
                    # Normalize angle to [-π, π] for comparison
                    while lidar_angle > math.pi:
                        lidar_angle -= 2 * math.pi
                    while lidar_angle < -math.pi:
                        lidar_angle += 2 * math.pi
                    
                    # Check if this beam is within our direction cone
                    # Handle angle wrapping for edge cases
                    angle_in_range = False
                    if start_angle <= end_angle:
                        # Normal case: range doesn't wrap
                        angle_in_range = start_angle <= lidar_angle <= end_angle
                    else:
                        # Wrapped case: range crosses -π/+π boundary
                        angle_in_range = (lidar_angle >= start_angle) or (lidar_angle <= end_angle)
                    
                    if angle_in_range:
                        valid_distances.append(dist)
                        if dist < min_dist_in_direction:
                            min_dist_in_direction = dist
                
                # Use the minimum distance as "clearance" for this direction
                # If no obstacle found, use max range
                if min_dist_in_direction == float("inf"):
                    min_dist_in_direction = self.lidar_max_range
                
                # Update best direction if this one is clearer
                if min_dist_in_direction > max_clear_distance:
                    max_clear_distance = min_dist_in_direction
                    best_angle = center_rad
            
            # If we found a good direction with enough clearance
            if max_clear_distance > obstacle_threshold:
                return best_angle, max_clear_distance
            
            # If all directions are blocked, return None to indicate need to backup/turn
            return None, None
            
        except Exception:
            pass

        return None, None

    def detect_stuck_situation(self, threshold=0.35):
        """
        Check if robot is stuck (obstacles very close on multiple sides).
        Returns (is_stuck, closest_distance) where closest_distance is the minimum distance
        to any obstacle in any direction.
        """
        if not self.lidar:
            return False, float("inf")
        
        try:
            range_image = self.lidar.getRangeImage()
            if not range_image or len(range_image) == 0:
                return False, float("inf")
            
            # Find the absolute closest obstacle in any direction
            closest_distance = float("inf")
            
            # Count how many directions have obstacles very close
            close_obstacle_count = 0
            directions_checked = 0
            
            angle_step = self.lidar_fov / len(range_image)
            
            # Check multiple sectors around the robot
            sectors = [
                ("front", 0, 45),
                ("left", -90, 30),
                ("right", 90, 30),
                ("back-left", -135, 30),
                ("back-right", 135, 30),
            ]
            
            for sector_name, center_deg, width_deg in sectors:
                center_rad = math.radians(center_deg)
                half_width_rad = math.radians(width_deg / 2.0)
                start_angle = center_rad - half_width_rad
                end_angle = center_rad + half_width_rad
                
                min_dist_in_sector = float("inf")
                
                for i, dist in enumerate(range_image):
                    if math.isnan(dist) or math.isinf(dist) or dist <= 0:
                        continue
                    
                    # Track absolute closest distance
                    if dist < closest_distance:
                        closest_distance = dist
                    
                    lidar_angle = -self.lidar_fov / 2.0 + i * angle_step
                    while lidar_angle > math.pi:
                        lidar_angle -= 2 * math.pi
                    while lidar_angle < -math.pi:
                        lidar_angle += 2 * math.pi
                    
                    angle_in_range = False
                    if start_angle <= end_angle:
                        angle_in_range = start_angle <= lidar_angle <= end_angle
                    else:
                        angle_in_range = (lidar_angle >= start_angle) or (lidar_angle <= end_angle)
                    
                    if angle_in_range and dist < min_dist_in_sector:
                        min_dist_in_sector = dist
                
                if min_dist_in_sector != float("inf"):
                    directions_checked += 1
                    if min_dist_in_sector < threshold:
                        close_obstacle_count += 1
            
            # Check if we're at or very near the lidar's minimum range (indicates pressed against wall)
            lidar_min_range = self.lidar_min_range if hasattr(self, 'lidar_min_range') else 0.20
            at_minimum_range = closest_distance <= (lidar_min_range + 0.05)  # Within 5cm of min range
            
            # If obstacles are close in 3+ directions, or if ANY obstacle is extremely close, or at min range, consider stuck
            is_stuck = close_obstacle_count >= 3 or closest_distance < 0.3 or at_minimum_range
            
            # If we didn't find any valid distances (all invalid), assume we might be stuck
            if closest_distance == float("inf"):
                # Check if we have many invalid readings (might indicate we're too close)
                invalid_count = sum(1 for d in range_image if math.isnan(d) or math.isinf(d) or d <= 0)
                if invalid_count > len(range_image) * 0.5:  # More than 50% invalid
                    is_stuck = True
                    closest_distance = lidar_min_range  # Assume we're at minimum range
            
            return is_stuck, closest_distance
            
        except Exception:
            return False, float("inf")

    def compute_motor_speeds(self):
        """Compute motor speeds with intelligent obstacle avoidance and exploration."""
        front_obstacle_dist, front_obstacle_angle = self.detect_front_obstacle()

        # Check if stuck (obstacles very close on multiple sides) - always back up first
        is_stuck, closest_distance = self.detect_stuck_situation(threshold=0.35)
        
        # If obstacle extremely close or stuck, back up first
        # Check if ANY obstacle is too close (including from detect_front_obstacle or closest_distance)
        any_obstacle_too_close = (front_obstacle_dist is not None and front_obstacle_dist < 0.25) or closest_distance < 0.25
        
        if any_obstacle_too_close or is_stuck:
            print(f"Robot stuck or obstacle too close (closest: {closest_distance:.2f}m, front: {front_obstacle_dist}) → backing up")
            # Find the best direction to back up toward (direction with most clearance)
            best_direction, clear_distance = self.find_best_exploration_direction(obstacle_threshold=0.3)
            
            backup_speed = -3.0  # Strong backup (increased from -2.5)
            turn_speed = 2.5
            
            # Use best direction if available, otherwise use front_obstacle_angle
            if best_direction is not None:
                # Back up toward the clearest direction (opposite of obstacle)
                left_speed = backup_speed - turn_speed if best_direction > 0 else backup_speed + turn_speed
                right_speed = backup_speed + turn_speed if best_direction > 0 else backup_speed - turn_speed
                print(f"  Backing up toward direction {math.degrees(best_direction):.1f}°, speeds: L={left_speed:.2f} R={right_speed:.2f}")
                return [left_speed, right_speed]
            elif front_obstacle_angle is not None:
                left_speed = backup_speed - turn_speed if front_obstacle_angle > 0 else backup_speed + turn_speed
                right_speed = backup_speed + turn_speed if front_obstacle_angle > 0 else backup_speed - turn_speed
                print(f"  Backing up based on front obstacle angle, speeds: L={left_speed:.2f} R={right_speed:.2f}")
                return [left_speed, right_speed]
            else:
                # Default: back up straight
                print(f"  Backing up straight, speeds: L={backup_speed:.2f} R={backup_speed:.2f}")
                return [backup_speed, backup_speed]

        # If obstacle close in front, find best direction to explore
        if front_obstacle_dist is not None and front_obstacle_dist < 0.4:
            best_direction, clear_distance = self.find_best_exploration_direction(obstacle_threshold=0.5)
            
            if best_direction is not None:
                # Check if the clearest path is actually clear enough to proceed forward
                # If clearance is too low, back up instead
                if clear_distance < 0.6:
                    print(f"Clearest path too close ({clear_distance:.2f}m) → backing up")
                    backup_speed = -1.5
                    turn_speed = 2.0
                    if best_direction > 0:
                        # Clear path on right, backup while turning right
                        return [backup_speed - turn_speed, backup_speed + turn_speed]
                    else:
                        # Clear path on left, backup while turning left
                        return [backup_speed + turn_speed, backup_speed - turn_speed]
                
                # Found a clear enough direction - steer toward it
                turn_intensity = abs(best_direction)
                base_forward = self.base_speed * 0.7  # Slower when avoiding
                
                # Determine turn direction
                if best_direction > 0:
                    # Turn right (toward right side)
                    left_speed = base_forward + turn_intensity * 3.0
                    right_speed = base_forward - turn_intensity * 3.0
                else:
                    # Turn left (toward left side)
                    left_speed = base_forward - turn_intensity * 3.0
                    right_speed = base_forward + turn_intensity * 3.0
                
                # Clamp speeds
                left_speed = max(-self.max_velocity, min(left_speed, self.max_velocity))
                right_speed = max(-self.max_velocity, min(right_speed, self.max_velocity))
                
                print(f"Obstacle detected → steering toward clearest path: {math.degrees(best_direction):.1f}° (clearance: {clear_distance:.2f}m)")
                return [left_speed, right_speed]
            else:
                # All directions blocked, backup and turn
                print("All directions blocked → backing up")
                backup_speed = -1.5
                turn_speed = 2.5
                if front_obstacle_angle is not None and front_obstacle_angle > 0:
                    # Obstacle on left, backup while turning right
                    return [backup_speed - turn_speed, backup_speed + turn_speed]
                else:
                    # Obstacle on right, backup while turning left
                    return [backup_speed + turn_speed, backup_speed - turn_speed]

        # No immediate obstacle, but check for approaching obstacles
        base_speeds = [self.base_speed, self.base_speed]

        avoidance_speed = [0.0, 0.0]
        if front_obstacle_dist is not None and front_obstacle_angle is not None:
            avoidance_strength = 8.0
            if front_obstacle_dist < 0.6:
                factor = (0.6 - front_obstacle_dist) / 0.6
                if front_obstacle_angle > 0:
                    # Obstacle on left, steer right
                    avoidance_speed[0] += factor * avoidance_strength
                    avoidance_speed[1] -= factor * avoidance_strength * 0.9
                else:
                    # Obstacle on right, steer left
                    avoidance_speed[0] -= factor * avoidance_strength * 0.9
                    avoidance_speed[1] += factor * avoidance_strength

        if front_obstacle_dist is not None and front_obstacle_dist < 0.50:
            motor_speed = [
                base_speeds[0] * 0.3 + avoidance_speed[0] * 1.2,
                base_speeds[1] * 0.3 + avoidance_speed[1] * 1.2,
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
        Sample depth at the bounding box centre and nearby points for robustness.
        Returns distance in meters or None if invalid.
        """
        range_finder = self.range_finder
        rgb_cam = self.camera_rgb
        if range_finder is None or rgb_cam is None:
            return None

        w = rgb_cam.getWidth()
        h = rgb_cam.getHeight()

        dw = range_finder.getWidth()
        dh = range_finder.getHeight()
        
        try:
            depth_img = range_finder.getRangeImage()
        except Exception as e:
            # RangeFinder might not be ready yet or method not available
            return None
            
        if depth_img is None or len(depth_img) == 0:
            return None

        # Map RGB → depth coordinates
        dx = int(cx * dw / w)
        dy = int(cy * dh / h)

        # Sample a small region around the center point for robustness
        sample_radius = 2
        valid_depths = []
        
        for offset_y in range(-sample_radius, sample_radius + 1):
            for offset_x in range(-sample_radius, sample_radius + 1):
                sample_x = dx + offset_x
                sample_y = dy + offset_y
                
                # Clamp to valid range
                sample_x = max(0, min(sample_x, dw - 1))
                sample_y = max(0, min(sample_y, dh - 1))

                # RangeFinder returns a 1D array, index is y * width + x
                idx = sample_y * dw + sample_x
                if idx >= len(depth_img):
                    continue
                    
                depth = depth_img[idx]

                # Filter out invalid values
                if (not math.isinf(depth) and not math.isnan(depth) and 
                    depth > 0.05 and depth < range_finder.getMaxRange()):
                    valid_depths.append(depth)
        
        if len(valid_depths) == 0:
            return None
        
        # Return median depth for robustness against outliers
        valid_depths.sort()
        median_depth = valid_depths[len(valid_depths) // 2]
        
        return median_depth
    
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
        Uses intelligent direction finding to navigate around obstacles.
        """
        left = forward
        right = forward

        if forward <= 0 or not self.lidar:
            return left, right

        front_obstacle_dist, front_obstacle_angle = self.detect_front_obstacle()

        if front_obstacle_dist is None:
            return left, right

        # Check if stuck - always back up first
        is_stuck, closest_distance = self.detect_stuck_situation(threshold=0.35)
        
        # If obstacle extremely close or stuck, back up first
        any_obstacle_too_close = (front_obstacle_dist is not None and front_obstacle_dist < 0.25) or closest_distance < 0.25
        
        if any_obstacle_too_close or is_stuck:
            print(f"Stuck or obstacle too close while following (closest: {closest_distance:.2f}m) → backing up")
            # Find best direction to back up toward
            best_direction, clear_distance = self.find_best_exploration_direction(obstacle_threshold=0.3)
            
            backup_speed = -2.5  # Strong backup
            turn_speed = 2.0
            
            if best_direction is not None:
                if best_direction > 0:
                    return [backup_speed - turn_speed, backup_speed + turn_speed]
                else:
                    return [backup_speed + turn_speed, backup_speed - turn_speed]
            elif front_obstacle_angle is not None and front_obstacle_angle > 0:
                return [backup_speed - turn_speed, backup_speed + turn_speed]
            else:
                return [backup_speed + turn_speed, backup_speed - turn_speed]

        # If obstacle is blocking the path, find best direction to navigate
        if front_obstacle_dist < 0.4:
            best_direction, clear_distance = self.find_best_exploration_direction(obstacle_threshold=0.5)
            
            if best_direction is not None:
                # Check if the clearest path is actually clear enough
                if clear_distance < 0.6:
                    print(f"Clearest path too close ({clear_distance:.2f}m) while following → backing up")
                    backup_speed = -1.5
                    turn_speed = 2.0
                    if best_direction > 0:
                        return [backup_speed - turn_speed, backup_speed + turn_speed]
                    else:
                        return [backup_speed + turn_speed, backup_speed - turn_speed]
                
                # Found a clear enough direction - steer toward it while maintaining some forward motion
                turn_intensity = abs(best_direction) * 2.0
                base_forward = forward * 0.5  # Reduce forward speed when avoiding
                
                if best_direction > 0:
                    # Turn right
                    left = base_forward + turn_intensity
                    right = base_forward - turn_intensity * 0.8
                else:
                    # Turn left
                    left = base_forward - turn_intensity * 0.8
                    right = base_forward + turn_intensity
                
                print(f"Obstacle blocking path → steering toward clearest direction: {math.degrees(best_direction):.1f}°")
            else:
                # All directions blocked, back up
                print("All directions blocked while following → backing up")
                backup_speed = -1.5
                turn_speed = 2.5
                if front_obstacle_angle is not None and front_obstacle_angle > 0:
                    return [backup_speed - turn_speed, backup_speed + turn_speed]
                else:
                    return [backup_speed + turn_speed, backup_speed - turn_speed]
        elif front_obstacle_dist < 0.5:
            # Obstacle approaching, gentle avoidance
            if front_obstacle_angle is not None:
                avoidance_factor = (0.5 - front_obstacle_dist) / 0.5
                if front_obstacle_angle > 0:
                    # Obstacle on left, steer right
                    left = forward + avoidance_factor * 1.5
                    right = forward - avoidance_factor * 1.2
                else:
                    # Obstacle on right, steer left
                    left = forward - avoidance_factor * 1.2
                    right = forward + avoidance_factor * 1.5

        # Clamp
        left = max(-self.max_velocity, min(left, self.max_velocity))
        right = max(-self.max_velocity, min(right, self.max_velocity))
        return left, right
    
    

    # TODO: Add a function to draw the SLAM map.

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
                # User detected
                print(f"User detected at ({cx},{cy}), depth={user_distance}")
                forward = self.compute_follow_speed(user_distance)
                left_speed, right_speed = self.compute_follow_motor_speeds(forward)
            else:
                # No user → pure obstacle avoidance based on lidar
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

            step_count += 1

        self.slam.save_map("slam_map_final.json")
        print("SLAM mapping complete!")


def main():
    controller = RosbotSlamController()
    if controller.robot:
        controller.run()


if __name__ == "__main__":
    main()
