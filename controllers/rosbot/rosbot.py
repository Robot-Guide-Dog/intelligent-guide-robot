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


class RosbotSlamController:
    def __init__(self):
        if not WEBOTS_AVAILABLE:
            print("Error: Webots not available")
            return

        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        self.slam = ParticleFilter(num_particles=100, map_size=200, resolution=0.05)

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

        # Stuck / escape bookkeeping
        self.stuck_ticks = 0
        self.wall_detected_ticks = 0  # Track consecutive wall detections
        self.escape_steps = 0
        self.escape_dir = -1  # -1 = left, 1 = right

        self.wheel_radius = 0.05
        self.wheel_base = 0.22

        self.base_speed = 2.5
        self.max_velocity = 20.0

        self.lidar_max_range = self.lidar.getMaxRange()
        self.lidar_min_range = self.lidar.getMinRange()

        print(
            f"Lidar enabled: {self.lidar_width} points, "
            f"FOV: {math.degrees(self.lidar_fov):.1f}°, "
            f"Range: {self.lidar_min_range:.2f}-{self.lidar_max_range:.2f}m"
        )


        print("Rosbot SLAM Controller initialized")
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
        """Detect obstacles in front of robot (wider forward arc)."""
        if not self.lidar:
            return None, None

        try:
            range_image = self.lidar.getRangeImage()
            if not range_image or len(range_image) == 0:
                return None, None

            angle_step = self.lidar_fov / len(range_image)

            # Widen front detection to 25-75% (50% of scan, ~180° forward arc)
            front_start_idx = int(len(range_image) * 0.25)
            front_end_idx = int(len(range_image) * 0.75)

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
    
    def detect_any_close_obstacle(self, threshold=0.5):
        """Check if ANY obstacle is within threshold distance in forward 180° arc."""
        if not self.lidar:
            return False, None

        try:
            range_image = self.lidar.getRangeImage()
            if not range_image or len(range_image) == 0:
                return False, None

            # Check forward 180° arc (25-75% of scan)
            front_start_idx = int(len(range_image) * 0.25)
            front_end_idx = int(len(range_image) * 0.75)

            for i in range(front_start_idx, front_end_idx):
                dist = range_image[i]
                if not (math.isnan(dist) or math.isinf(dist) or dist <= 0):
                    if dist < threshold:
                        return True, dist
        except Exception:
            pass

        return False, None

    def compute_motor_speeds(self):
        """Compute motor speeds with obstacle avoidance."""
        # Triggers an escape if a wall is detected first. If not, it will continue to follow the wall.
        escape = self.escape_command()
        if escape:
            return escape

        front_obstacle_dist, front_obstacle_angle = self.detect_front_obstacle()
        
        has_close_obstacle, close_dist = self.detect_any_close_obstacle(threshold=0.5)
        
        # If any obstacle's very close, it will trigger an escape instead of just stopping
        if has_close_obstacle and close_dist is not None and close_dist < 0.25:
            self.wall_detected_ticks += 1
            left_wall, right_wall = self.detect_side_walls()
            
            # Determines which direction to turn (toward more space)
            turn_left = True
            if left_wall is not None and right_wall is not None:
                turn_left = left_wall > right_wall
            elif front_obstacle_angle is not None:
                # Turn away from the obstacle
                turn_left = front_obstacle_angle < 0
            else:
                turn_left = random.random() > 0.5
            
            # Trigger escape immediately if wall detected multiple times, or after first detection
            if self.wall_detected_ticks >= 2 or self.escape_steps == 0:
                self.trigger_escape(turn_left, reason=f"WALL DETECTED at {close_dist:.2f}m")
                return self.escape_command()
            else:
                print(f"WALL DETECTED at {close_dist:.2f}m → backing up")
                if turn_left:
                    return [-2.0, 1.5]  # Back left, forward right
                else:
                    return [1.5, -2.0]  # Forward left, back right
        else:
            self.wall_detected_ticks = 0

        self.reverse_if_needed(front_obstacle_dist)

        left_wall, right_wall = self.detect_side_walls()

        if front_obstacle_dist is not None and front_obstacle_dist < 0.32:
            self.stuck_ticks += 1
        else:
            self.stuck_ticks = 0

        if (
            self.stuck_ticks > 8
            or (
                left_wall is not None
                and right_wall is not None
                and left_wall < 0.35
                and right_wall < 0.35
                and front_obstacle_dist is not None
                and front_obstacle_dist < 0.45
            )
            or (front_obstacle_dist is not None and front_obstacle_dist < 0.18)
        ):
            turn_left = True
            if left_wall is not None and right_wall is not None:
                turn_left = left_wall > right_wall
            else:
                turn_left = random.random() > 0.5

            self.trigger_escape(turn_left, reason="Corner detected")
            return self.escape_command()

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

        # Slow down significantly when approaching obstacles
        if front_obstacle_dist is not None and front_obstacle_dist < 0.35:
            if front_obstacle_angle is not None:
                turn_speed = 3.0
                if front_obstacle_angle > 0:
                    return [turn_speed, -turn_speed * 0.1]
                return [-turn_speed * 0.1, turn_speed]
            return [0.0, 0.0]

        # Reduce base speed when obstacles are detected nearby
        base_speeds = [self.base_speed, self.base_speed]
        
        # If any close obstacle detected, reduce speed
        if has_close_obstacle and close_dist is not None:
            if close_dist < 0.5:
                base_speeds = [self.base_speed * 0.3, self.base_speed * 0.3]  # Slow down
            elif close_dist < 0.7:
                base_speeds = [self.base_speed * 0.6, self.base_speed * 0.6]  # Moderate speed

        avoidance_speed = [0.0, 0.0]
        if front_obstacle_dist is not None and front_obstacle_angle is not None:
            avoidance_strength = 8.0
            if front_obstacle_dist < 0.5:  # Increased from 0.4 to react earlier
                factor = (0.5 - front_obstacle_dist) / 0.5
                if front_obstacle_angle > 0:
                    avoidance_speed[0] += factor * avoidance_strength
                    avoidance_speed[1] -= factor * avoidance_strength * 0.9
                else:
                    avoidance_speed[0] -= factor * avoidance_strength * 0.9
                    avoidance_speed[1] += factor * avoidance_strength

        if front_obstacle_dist is not None and front_obstacle_dist < 0.50:  # Increased from 0.40
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


    # Setting the motor velocities based on the left and right speeds given by the compute_motor_speeds function
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

    # STUCK
    def reverse_if_needed(self, dist):
        """
        Reverse if an obstacle is closer than 1.5 meters.
        """
        if dist is None:
            return

        if dist < 0.15:
            print(f"Obstacle at {dist:.2f}m → reversing")
            for _ in range(15):  # about 150–200 ms
                self.set_motor_velocities(-3.0, -3.0)  # reverse speed
                self.robot.step(self.time_step)

            print("Reverse complete")

    def trigger_escape(self, turn_left=True, reason="stuck"):
        """
        Schedule a multi-step escape turn to break out of tight corners.
        """
        self.escape_steps = max(self.escape_steps, 15)
        self.escape_dir = -1 if turn_left else 1
        side = "LEFT" if turn_left else "RIGHT"
        print(f"{reason} → escape {side} for {self.escape_steps} steps")

    def escape_command(self):
        """
        If an escape maneuver is active, emit the pre-set turn command.
        """
        if self.escape_steps <= 0:
            return None

        self.escape_steps -= 1
        spin = 3.5
        back = -2.0

        if self.escape_dir < 0:  # turn left
            return back, spin

        return spin, back

    def detect_side_walls(self):
        """Return (left_min_dist, right_min_dist) from lidar."""
        if not self.lidar:
            return None, None

        ranges = self.lidar.getRangeImage()
        if not ranges:
            return None, None

        n = len(ranges)

        # Left = first 20% of lidar
        left_ranges = ranges[:int(n * 0.2)]

        # Right = last 20% of lidar
        right_ranges = ranges[int(n * 0.8):]

        # Get nearest valid hit
        left_min = min(
            [d for d in left_ranges if d > 0 and math.isfinite(d)],
            default=5.0,
        )
        right_min = min(
            [d for d in right_ranges if d > 0 and math.isfinite(d)],
            default=5.0,
        )

        return left_min, right_min


    # COLOR + DEPTH DETECTION: NAYLEA
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

        # Map RGB: depth coordinates
        dx = int(cx * dw / w)
        dy = int(cy * dh / h)

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

    # The func computes the forward speed based on the distance to the user and the speed will be 0 if the user is too far or too close
    # Essentailly to mimic a guide dog on a lead of whether it is pulling or not
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
        escape = self.escape_command()
        if escape:
            return escape

        left = forward
        right = forward

        if forward <= 0 or not self.lidar:
            return left, right

        front_obstacle_dist, front_obstacle_angle = self.detect_front_obstacle()
        
        # Check for ANY close obstacle in forward arc (safety check)
        has_close_obstacle, close_dist = self.detect_any_close_obstacle(threshold=0.5)
        
        # If any obstacle is very close, trigger escape instead of just stopping
        if has_close_obstacle and close_dist is not None and close_dist < 0.25:
            self.wall_detected_ticks += 1
            left_wall, right_wall = self.detect_side_walls()
            
            # Determine which direction to turn (toward more clearance)
            turn_left = True
            if left_wall is not None and right_wall is not None:
                turn_left = left_wall > right_wall
            elif front_obstacle_angle is not None:
                # Turn away from the obstacle
                turn_left = front_obstacle_angle < 0
            else:
                turn_left = random.random() > 0.5
            
            # Trigger escape immediately if wall detected multiple times, or after first detection
            if self.wall_detected_ticks >= 2 or self.escape_steps == 0:
                self.trigger_escape(turn_left, reason=f"WALL DETECTED at {close_dist:.2f}m")
                return self.escape_command()
            else:
                # First detection - back up and turn
                print(f"WALL DETECTED at {close_dist:.2f}m → backing up")
                if turn_left:
                    return [-2.0, 1.5]  # Back left, forward right
                else:
                    return [1.5, -2.0]  # Forward left, back right
        else:
            self.wall_detected_ticks = 0
        
        left_wall, right_wall = self.detect_side_walls()

        # Track thr "stuck" condition when an obstacle is repeatedly close.
        if front_obstacle_dist is not None and front_obstacle_dist < 0.32:
            self.stuck_ticks += 1
        else:
            self.stuck_ticks = 0

        # If the robot is stuck, it will turn away from the obstacle
        if (
            self.stuck_ticks > 8
            or (
                left_wall is not None
                and right_wall is not None
                and left_wall < 0.35
                and right_wall < 0.35
                and front_obstacle_dist is not None
                and front_obstacle_dist < 0.45
            )
            or (front_obstacle_dist is not None and front_obstacle_dist < 0.18)
        ):
            turn_left = True
            if left_wall is not None and right_wall is not None:
                # Turn toward the side with more clearance
                turn_left = left_wall > right_wall
            else:
                turn_left = random.random() > 0.5

            self.trigger_escape(turn_left, reason="Corner detected" if self.stuck_ticks > 0 else "Obstacle extremely close")
            return self.escape_command()

    # Basically checks if there is an obstacle in front of the robot and if there is, it will turn away from it
        if front_obstacle_dist is None:
            if has_close_obstacle and close_dist is not None and close_dist < 0.5:
                left *= 0.3
                right *= 0.3
            return left, right

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

        if front_obstacle_dist < 0.3:
            if front_obstacle_angle is not None and front_obstacle_angle > 0:
                print("Obstacle on RIGHT → turn LEFT")
                left = 0
                right = 2.5
            else:
                print("Obstacle on LEFT → turn RIGHT")
                left = 2.5
                right = 0
            return left, right

        # Reduce forward speed when obstacles are nearby
        if has_close_obstacle and close_dist is not None:
            if close_dist < 0.5:
                left *= 0.3  # Slow down significantly
                right *= 0.3
            elif close_dist < 0.7:
                left *= 0.6  # Moderate speed
                right *= 0.6
        left = max(-self.max_velocity, min(left, self.max_velocity))
        right = max(-self.max_velocity, min(right, self.max_velocity))
        return left, right
    
    # Draws the SLAM map on the display  (Not used in the final implementation)
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

    
    # Runs the main loop that controls the particle filter SLAM and the user following algorithm
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
                left_speed, right_speed = self.compute_motor_speeds()

            self.set_motor_velocities(left_speed, right_speed)

            # 2) SLAM update (odometry + lidar)
            dx, dy, dtheta = self.compute_odometry()
            self.slam.predict(dx, dy, dtheta)

            if step_count % 3 == 0:     
                self.draw_slam_map()

            if step_count % 5 == 0:
                self.process_lidar()
                resample_counter += 1
                if resample_counter >= 10:
                    self.slam.resample()
                    resample_counter = 0

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