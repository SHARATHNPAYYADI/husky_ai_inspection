from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'husky_waypoint_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sharathnpayyadi',
    maintainer_email='sharathnp1998@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_waypoints = husky_waypoint_recorder.record_waypoints:main',
            'follow_waypoints = husky_waypoint_recorder.follow_waypoints:main'
        ],
    },
)
