import os
from glob import glob
from setuptools import setup

package_name = 'sim_waypoint_path'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={package_name: 'sim_waypoint_path'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include all launch files
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Channing Ko-Madden',
    maintainer_email='channingkomadden@gmail.com',
    description='Tools for creating and visualizing paths',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz_point_visualizer = sim_waypoint_path.rviz_point_visualizer:main',
            'waypoint_follower = sim_waypoint_path.waypoint_follower:main',
        ],
    },
)
