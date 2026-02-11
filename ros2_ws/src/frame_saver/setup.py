import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'frame_saver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='icta',
    maintainer_email='hoang.nguyen@framatome.com',
    description='Package to save ROS2 images to disk',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'frame_saver_node = frame_saver.frame_saver_node:main',
            'dummy_publisher_node = frame_saver.dummy_publisher:main',
        ],
    },
)
