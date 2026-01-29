from setuptools import find_packages, setup

package_name = 'husky_vision_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'detector_node = husky_vision_detector.detector_node:main',
            'yolo_fire_extinguisher_node = husky_vision_detector.yolo_fire_extinguisher_node:main',
            'image_capture_service = husky_vision_detector.image_capture_service:main',
        ],
    },
)
