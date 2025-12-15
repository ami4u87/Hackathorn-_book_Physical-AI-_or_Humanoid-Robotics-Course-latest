from setuptools import setup
import os
from glob import glob

package_name = 'tf2_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='ROS 2 TF2 (Transform Library) example for Physical AI course',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf2_broadcaster = tf2_example.broadcaster_node:main',
            'tf2_listener = tf2_example.listener_node:main',
        ],
    },
)