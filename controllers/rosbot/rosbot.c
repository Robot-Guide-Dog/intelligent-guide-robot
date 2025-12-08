/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gyro.h>
#include <webots/lidar.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>

#define TIME_STEP 32
#define MAX_VELOCITY 20

void rgb_to_hsv(unsigned char r, unsigned char g, unsigned char b,
                float *h, float *s, float *v) {

  float rf = r / 255.0;
  float gf = g / 255.0;
  float bf = b / 255.0;

  float max = rf;
  if (gf > max) max = gf;
  if (bf > max) max = bf;

  float min = rf;
  if (gf < min) min = gf;
  if (bf < min) min = bf;

  float delta = max - min;

  // Value
  *v = max;

  // Saturation
  if (max == 0)
    *s = 0;
  else
    *s = delta / max;

  // Hue calculation
  if (delta == 0) {
    *h = 0;
  } else if (max == rf) {
    *h = 60 * fmod(((gf - bf) / delta), 6);
  } else if (max == gf) {
    *h = 60 * (((bf - rf) / delta) + 2);
  } else {
    *h = 60 * (((rf - gf) / delta) + 4);
  }

  if (*h < 0)
    *h += 360;
}

bool bound_box(WbDeviceTag camera_rgb,
                             int *min_x, int *min_y,
                             int *max_x, int *max_y,
                             int *pixel_count)
{
  int w = wb_camera_get_width(camera_rgb);
  int h = wb_camera_get_height(camera_rgb);
  const unsigned char *img = wb_camera_get_image(camera_rgb);

  if (!img) return false;

  *min_x = w; *min_y = h;
  *max_x = 0; *max_y = 0;
  *pixel_count = 0;

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {

      unsigned char r = wb_camera_image_get_red(img, w, x, y);
      unsigned char g = wb_camera_image_get_green(img, w, x, y);
      unsigned char b = wb_camera_image_get_blue(img, w, x, y);

      float H, S, V;
      rgb_to_hsv(r, g, b, &H, &S, &V);

      bool hsv_green =
        (H > 50 && H < 150) &&
        (S > 0.30) &&
        (V > 0.25);

      bool rgb_green =
        (g > r + 40 && g > b + 40);

      if (!(hsv_green || rgb_green))
        continue;

      if (x < *min_x) *min_x = x;
      if (x > *max_x) *max_x = x;
      if (y < *min_y) *min_y = y;
      if (y > *max_y) *max_y = y;

      (*pixel_count)++;
    }
  }

  return (*pixel_count > 100);  // must detect enough pixels

}

int main(int argc, char *argv[]) {
  /* define variables */
  /* motors */
  WbDeviceTag front_left_motor, front_right_motor, rear_left_motor, rear_right_motor, front_left_position_sensor,
    front_right_position_sensor, rear_left_position_sensor, rear_right_position_sensor;
  double avoidance_speed[2];
  const double base_speed = 2.5;
  double motor_speed[2];
  /* RGBD camera */
  WbDeviceTag camera_rgb, camera_depth;
  /* rotational lidar */
  WbDeviceTag lidar;
  /* IMU */
  WbDeviceTag accelerometer, gyro, compass;
  /* distance sensors */
  WbDeviceTag distance_sensors[4];
  double distance_sensors_value[4];

  // set empirical coefficients for collision avoidance
  const double coefficients[2][2] = {{15.0, -9.0}, {-15.0, 9.0}};

  int i, j;

  /* initialize Webots */
  wb_robot_init();

  /* get a handler to the motors and set target position to infinity (speed control). */
  front_left_motor = wb_robot_get_device("fl_wheel_joint");
  front_right_motor = wb_robot_get_device("fr_wheel_joint");
  rear_left_motor = wb_robot_get_device("rl_wheel_joint");
  rear_right_motor = wb_robot_get_device("rr_wheel_joint");
  wb_motor_set_position(front_left_motor, INFINITY);
  wb_motor_set_position(front_right_motor, INFINITY);
  wb_motor_set_position(rear_left_motor, INFINITY);
  wb_motor_set_position(rear_right_motor, INFINITY);
  wb_motor_set_velocity(front_left_motor, 0.0);
  wb_motor_set_velocity(front_right_motor, 0.0);
  wb_motor_set_velocity(rear_left_motor, 0.0);
  wb_motor_set_velocity(rear_right_motor, 0.0);

  /* get a handler to the position sensors and enable them. */
  front_left_position_sensor = wb_robot_get_device("front left wheel motor sensor");
  front_right_position_sensor = wb_robot_get_device("front right wheel motor sensor");
  rear_left_position_sensor = wb_robot_get_device("rear left wheel motor sensor");
  rear_right_position_sensor = wb_robot_get_device("rear right wheel motor sensor");
  wb_position_sensor_enable(front_left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(front_right_position_sensor, TIME_STEP);
  wb_position_sensor_enable(rear_left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(rear_right_position_sensor, TIME_STEP);

  /* get a handler to the ASTRA rgb and depth cameras and enable them. */
  camera_rgb = wb_robot_get_device("camera rgb");
  camera_depth = wb_robot_get_device("camera depth");
  wb_camera_enable(camera_rgb, TIME_STEP);
  wb_range_finder_enable(camera_depth, TIME_STEP);

  /* get a handler to the RpLidarA2 and enable it. */
  lidar = wb_robot_get_device("laser");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);

  /* get a handler to the IMU devices and enable them. */
  accelerometer = wb_robot_get_device("imu accelerometer");
  gyro = wb_robot_get_device("imu gyro");
  compass = wb_robot_get_device("imu compass");
  wb_accelerometer_enable(accelerometer, TIME_STEP);
  wb_gyro_enable(gyro, TIME_STEP);
  wb_compass_enable(compass, TIME_STEP);

  /* get a handler to the distance sensors and enable them. */
  distance_sensors[0] = wb_robot_get_device("fl_range");
  distance_sensors[1] = wb_robot_get_device("rl_range");
  distance_sensors[2] = wb_robot_get_device("fr_range");
  distance_sensors[3] = wb_robot_get_device("rr_range");
  wb_distance_sensor_enable(distance_sensors[0], TIME_STEP);
  wb_distance_sensor_enable(distance_sensors[1], TIME_STEP);
  wb_distance_sensor_enable(distance_sensors[2], TIME_STEP);
  wb_distance_sensor_enable(distance_sensors[3], TIME_STEP);

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    /* get accelerometer values */
    const double *a = wb_accelerometer_get_values(accelerometer);
    printf("accelerometer values = %0.2f %0.2f %0.2f\n", a[0], a[1], a[2]);

    /* get distance sensors values */
    for (i = 0; i < 4; i++)
      distance_sensors_value[i] = wb_distance_sensor_get_value(distance_sensors[i]);


    int min_x, min_y, max_x, max_y, green_pixels;

    bool found = bound_box(camera_rgb,
                                 &min_x, &min_y,
                                 &max_x, &max_y,
                                 &green_pixels);

    printf("\nGreen pixels = %d\n", green_pixels);

    if (!found) {
      printf("No user detected.\n");

      wb_motor_set_velocity(front_left_motor, 0);
      wb_motor_set_velocity(front_right_motor, 0);
      wb_motor_set_velocity(rear_left_motor, 0);
      wb_motor_set_velocity(rear_right_motor, 0);
      continue;
    }

    printf("User BOX = (%d,%d) to (%d,%d)\n",
           min_x, min_y, max_x, max_y);


    // ---- Compute bounding box center ----
    int cx = (min_x + max_x) / 2;
    int cy = (min_y + max_y) / 2;
    printf("User center pixel = (%d, %d)\n", cx, cy);


    // ---- Depth lookup at bounding box center ----
    int w = wb_camera_get_width(camera_rgb);
    int h = wb_camera_get_height(camera_rgb);

    int dw = wb_range_finder_get_width(camera_depth);
    int dh = wb_range_finder_get_height(camera_depth);
    const float *depth_img = wb_range_finder_get_range_image(camera_depth);

    int dx = (cx * dw) / w;
    int dy = (cy * dh) / h;

    float user_distance = depth_img[dy * dw + dx];

    printf("User distance = %.3f m\n", user_distance);

    if (isinf(user_distance) || user_distance < 0.6) {
    // too close → depth fails
    printf("User too close for depth! Using fallback (0.3 m).\n");
    user_distance = 0.3; // treat as close
    }

    // -------- FOLLOWING BEHAVIOR --------
    float forward = 0;

    float d_close = 0.25;
    float d_far = 3.0;

    float min_spd = 0.5;
    float max_spd = 3.0;

    if (user_distance < d_close)
      forward = max_spd;
    else if (user_distance > d_far)
      forward = 0;
    else {
      float ratio = 1.0 - ((user_distance - d_close) /
                           (d_far - d_close));
      forward = min_spd + ratio * (max_spd - min_spd);
    }

    printf("Forward speed = %.2f\n", forward);

    // -------- OBSTACLE AVOIDANCE (LIDAR) --------
    float left = forward;
    float right = forward;

    int lw = wb_lidar_get_horizontal_resolution(lidar);
    const float *lv = wb_lidar_get_range_image(lidar);

    int mid = lw / 2;
    int li = mid - 20; if (li < 0) li = 0;
    int ri = mid + 20; if (ri >= lw) ri = lw - 1;

    float L = lv[li];
    float C = lv[mid];
    float R = lv[ri];

    printf("Lidar L=%.2f C=%.2f R=%.2f\n", L, C, R);

    if (forward > 0) {
      if (C < 0.4) {
        printf("Obstacle ahead → rotate\n");
        left = 2.5;
        right = -2.5;
      }
      else if (L < 0.3) {
        printf("Obstacle left → turn right\n");
        left = 2.5;
        right = 0.5;
      }
      else if (R < 0.3) {
        printf("Obstacle right → turn left\n");
        left = 0.5;
        right = 2.5;
      }
    }

    // Motor clamp
    if (left > MAX_VELOCITY) left = MAX_VELOCITY;
    if (right > MAX_VELOCITY) right = MAX_VELOCITY;
    if (left < -MAX_VELOCITY) left = -MAX_VELOCITY;
    if (right < -MAX_VELOCITY) right = -MAX_VELOCITY;

    wb_motor_set_velocity(front_left_motor, left);
    wb_motor_set_velocity(rear_left_motor, left);
    wb_motor_set_velocity(front_right_motor, right);
    wb_motor_set_velocity(rear_right_motor, right);

  }

  wb_robot_cleanup();

  return 0;
}
