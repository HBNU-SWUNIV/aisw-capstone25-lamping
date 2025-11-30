from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'turtlebot3_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aisw',
    maintainer_email='aisw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = turtlebot3_motion.yolo_node:main',
            'static_obstacle_node = turtlebot3_motion.static_obstacle_node:main',
            'warning_sound_node = turtlebot3_motion.warning_sound_node:main',
            'camera_compressor_node = turtlebot3_motion.camera_compressor_node:main',
			'camera_bridge = turtlebot3_motion.sim_camera_bridge:main',
			'camera_check = turtlebot3_motion.camera_check:main',
			'tb3_bookmark_bridge = turtlebot3_motion.tb3_bookmark_bridge:main',
        ],
    },
)
