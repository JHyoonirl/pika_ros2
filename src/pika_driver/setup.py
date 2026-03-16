import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pika_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irl',
    maintainer_email='face5921@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fisheye_driver = pika_driver.fisheye_driver:main',
            'motor_driver = pika_driver.motor_driver:main',
            'vive_tracker_driver = pika_driver.vive_tracker_driver:main',
            'realsense_driver = pika_driver.realsense_driver:main',
        ],
    },
)
