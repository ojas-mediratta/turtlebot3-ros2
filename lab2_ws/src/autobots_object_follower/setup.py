# ~/lab2_ws/src/autobots_object_follower/setup.py

from setuptools import find_packages, setup
import os  # <-- THIS LINE WAS MISSING
from glob import glob

package_name = 'autobots_object_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='burger',
    maintainer_email='burger@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_object = autobots_object_follower.find_object_node:main',
            'rotate_robot = autobots_object_follower.rotate_robot_node:main',
        ],
    },
)