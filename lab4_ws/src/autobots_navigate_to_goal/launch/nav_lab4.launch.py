#!/usr/bin/env python3
# nav_lab4.launch.py
# Lab 4 Master Launch File
# Launches both navigation nodes (get_object_range and go_to_goal)

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for Lab 4 autonomous navigation.
    
    Assumes turtlebot3_bringup is already running separately:
        ros2 launch turtlebot3_bringup robot.launch.py
    
    This launch file starts:
        1. get_object_range - LIDAR obstacle detection
        2. go_to_goal - Waypoint navigation with obstacle avoidance
    """
    
    return LaunchDescription([
        
        # Node 1: Obstacle detection from LIDAR
        Node(
            package='autobots_navigate_to_goal',
            executable='get_object_range',
            name='get_object_range',
            output='screen',
            emulate_tty=True,
        ),
        
        # Node 2: Navigation controller
        Node(
            package='autobots_navigate_to_goal',
            executable='go_to_goal',
            name='go_to_goal',
            output='screen',
            emulate_tty=True,
        ),
    ])
