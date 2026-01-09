from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'road_segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # INSTALL MODELS (best.pt)
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nidhirajr',
    maintainer_email='nidhirajr@gmail.com',
    description='Road segmentation using YOLOv8',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'road_seg_node = road_segmentation.road_seg_node:main',
        ],
    },
)
