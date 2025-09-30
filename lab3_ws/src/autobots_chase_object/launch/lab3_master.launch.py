# ~/autobots/lab3_ws/src/autobots_chase_object/launch/lab3_master.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the robot bringup launch file
    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    camera_robot_launch_file = os.path.join(turtlebot3_bringup_dir, 'launch', 'camera_robot.launch.py')

    return LaunchDescription([
        # 1. Include the robot and camera bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_robot_launch_file)
        ),
        # 2. Start the detect_object node
        Node(
            package='autobots_chase_object',
            executable='detect_object',
            name='detect_object_node'
        ),
        # 3. Start the get_object_range node
        Node(
            package='autobots_chase_object',
            executable='get_object_range',
            name='get_object_range_node'
        ),
        # 4. Start the chase_object node
        Node(
            package='autobots_chase_object',
            executable='chase_object',
            name='chase_object_node'
        ),
    ])