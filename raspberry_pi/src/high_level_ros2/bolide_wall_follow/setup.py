import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bolide_wall_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='voiture',
    maintainer_email='minh_nhut.nguyen@etu.sorbonne-universite.fr',
    description='Wall Follow package with a lidar debug node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follow=bolide_wall_follow.wall_follow_node:main',
            'wall_follow_pid=bolide_wall_follow.wall_follow_pid:main',
            'lidar_front_only=bolide_wall_follow.lidar_front_only:main',
            'follow_gap=bolide_wall_follow.follow_gap:main',
            'save_scan=bolide_wall_follow.save_scan:main' ,
            # 'auto_map=bolide_wall_follow:auto_map:main',
        ],
    },
)
