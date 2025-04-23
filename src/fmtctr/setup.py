from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fmtctr'

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
    maintainer='zenos',
    maintainer_email='ZenosZeng@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_subscriber = src.test_subscriber:main',
            'sensor_node = src.sensor_node:main',
            'control_node = src.control_node:main',
        ],
    },
)
