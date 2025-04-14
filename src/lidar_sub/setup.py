from setuptools import setup

package_name = 'lidar_sub'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco',
    maintainer_email='irotaicnoc@gmail.com',
    description='lidar subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_subscriber_node = lidar_sub.lidar_subscriber:main',
        ],
    },
)
