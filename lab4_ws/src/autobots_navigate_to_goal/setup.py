from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autobots_navigate_to_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include wayPoints.txt
        (os.path.join('share', package_name), [os.path.join(package_name, 'wayPoints.txt')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='burger',
    maintainer_email='omediratta3@gatech.edu',
    description='Lab 4: Autonomous waypoint navigation with obstacle avoidance for TurtleBot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_object_range = autobots_navigate_to_goal.get_object_range:main',
            'go_to_goal = autobots_navigate_to_goal.go_to_goal:main',
        ],
    },
)
