# ~/lab2_ws/src/autobots_object_follower/launch/autobots_master.launch.py

# to launch
# Don't forget to source your workspace first!
# source ~/lab2_ws/install/setup.bash

# Run everything with the new master launch file
# ros2 launch autobots_object_follower autobots_master.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Path to the launch file for camera and robot bringup
    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    camera_robot_launch_file = os.path.join(turtlebot3_bringup_dir, 'launch', 'camera_robot.launch.py')

    return LaunchDescription([
        # 1. Include the robot and camera bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_robot_launch_file)
        ),

        # 2. Add your find_object node
        Node(
            package='autobots_object_follower',
            executable='find_object',
            name='find_object_node',
            output='screen'
        ),

        # 3. Add your rotate_robot node
        Node(
            package='autobots_object_follower',
            executable='rotate_robot',
            name='rotate_robot_node',
            output='screen'
        ),
    ])