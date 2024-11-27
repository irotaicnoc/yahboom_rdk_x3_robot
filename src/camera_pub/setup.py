from setuptools import setup
import os
from glob import glob

package_name = 'camera_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marco',
    maintainer_email='irotaicnoc@gmail.com',
    description='camera publisher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher_node = camera_pub.camera_publisher:main',
        ],
    },
)
