# ~/lab2_ws/src/autobots_object_follower/launch/object_follower.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autobots_object_follower',
            executable='find_object',
            name='find_object_node',
            output='screen'
        ),
        Node(
            package='autobots_object_follower',
            executable='rotate_robot',
            name='rotate_robot_node',
            output='screen'
        ),
    ])