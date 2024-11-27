from setuptools import setup

package_name = 'controller_sub'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco',
    maintainer_email='irotaicnoc@gmail.com',
    description='controller subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_subscriber_node = controller_sub.controller_subscriber:main',
        ],
    },
)
