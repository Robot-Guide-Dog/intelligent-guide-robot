#!/usr/bin/env python3
"""
Launch file for SLAM with Webots Rosbot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get package path
    pkg_path = FindPackageShare('webots_slam')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    slam_config_arg = DeclareLaunchArgument(
        'slam_config',
        default_value=PathJoinSubstitution([
            pkg_path, 'config', 'slam_toolbox_params.yaml'
        ]),
        description='Path to SLAM configuration file'
    )
    
    # Robot state publisher (for tf tree)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': '''<?xml version="1.0"?>
<robot name="rosbot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
    </visual>
  </link>
  <link name="laser_frame"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>
  <link name="imu_frame"/>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_frame"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>'''
        }]
    )
    
    # SLAM Toolbox node
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            LaunchConfiguration('slam_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    # RViz for visualization (optional)
    rviz_config = PathJoinSubstitution([
        pkg_path, 'config', 'slam.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(EnvironmentVariable('LAUNCH_RVIZ', default_value='false'))
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        robot_state_publisher,
        slam_toolbox,
        rviz,
    ])



