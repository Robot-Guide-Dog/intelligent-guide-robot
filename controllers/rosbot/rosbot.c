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

bool is_user_in_view(WbDeviceTag camera) {
  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);
  const unsigned char *image = wb_camera_get_image(camera);

  if (!image)
    return false;

  int match_count = 0;

  for (int y = 0; y < height; y += 2) {
    for (int x = 0; x < width; x += 2) {
      unsigned char r = wb_camera_image_get_red(image, width, x, y);
      unsigned char g = wb_camera_image_get_green(image, width, x, y);
      unsigned char b = wb_camera_image_get_blue(image, width, x, y);

      float h, s, v;
      rgb_to_hsv(r, g, b, &h, &s, &v);

      bool is_green =
        (h > 60 && h < 170) &&   // Green hue
        (s > 0.35) &&            // Saturated
        (v > 0.10);              // Allow shadows

      if (is_green)
        match_count++;
    }
  }

  printf("HSV match_count = %d\n", match_count);

  return match_count > 40;
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


   // Debug center pixel HSV
    int w = wb_camera_get_width(camera_rgb);
    int h = wb_camera_get_height(camera_rgb);
    const unsigned char *img = wb_camera_get_image(camera_rgb);

    if (img && w > 0 && h > 0) {
      int cx = w / 2;
      int cy = h / 2;
      unsigned char r = wb_camera_image_get_red(img, w, cx, cy);
      unsigned char g = wb_camera_image_get_green(img, w, cx, cy);
      unsigned char b = wb_camera_image_get_blue(img, w, cx, cy);

      float h_val, s_val, v_val;
      rgb_to_hsv(r, g, b, &h_val, &s_val, &v_val);

      printf("Center pixel HSV = H: %.1f  S: %.2f  V: %.2f\n",
             h_val, s_val, v_val);
    }

    bool user_detected = is_user_in_view(camera_rgb);

    if (user_detected) {
      printf("User detected -> moving\n");

      // Simple forward movement for now
      wb_motor_set_velocity(front_left_motor, 2.0);
      wb_motor_set_velocity(front_right_motor, 2.0);
      wb_motor_set_velocity(rear_left_motor, 2.0);
      wb_motor_set_velocity(rear_right_motor, 2.0);

    } else {
      printf("No user detected -> stopping\n");

      wb_motor_set_velocity(front_left_motor, 0);
      wb_motor_set_velocity(front_right_motor, 0);
      wb_motor_set_velocity(rear_left_motor, 0);
      wb_motor_set_velocity(rear_right_motor, 0);
    }
    
    // /* compute motors speed */
    // for (i = 0; i < 2; ++i) {
    //   avoidance_speed[i] = 0.0;
    //   for (j = 1; j < 3; ++j)
    //     avoidance_speed[i] += (2.0 - distance_sensors_value[j]) * (2.0 - distance_sensors_value[j]) * coefficients[i][j - 1];
    //   motor_speed[i] = base_speed + avoidance_speed[i];
    //   motor_speed[i] = motor_speed[i] > MAX_VELOCITY ? MAX_VELOCITY : motor_speed[i];
    // }

    // /* set speed values */
    // wb_motor_set_velocity(front_left_motor, motor_speed[0]);
    // wb_motor_set_velocity(front_right_motor, motor_speed[1]);
    // wb_motor_set_velocity(rear_left_motor, motor_speed[0]);
    // wb_motor_set_velocity(rear_right_motor, motor_speed[1]);
  }

  wb_robot_cleanup();

  return 0;
}
