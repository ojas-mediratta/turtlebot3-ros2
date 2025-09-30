from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autobots_chase_object'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='burger',
    maintainer_email='burger@todo.todo',
    description='A ROS 2 package to make a TurtleBot3 chase an object using camera and LIDAR data.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_object = autobots_chase_object.detect_object_node:main',
            'get_object_range = autobots_chase_object.get_object_range_node:main',
            'chase_object = autobots_chase_object.chase_object_node:main',
        ],
    },
)