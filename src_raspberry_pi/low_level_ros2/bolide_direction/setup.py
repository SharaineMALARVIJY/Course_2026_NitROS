from setuptools import find_packages, setup

package_name = 'bolide_direction'

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
    maintainer='bolide team',
    maintainer_email='baptiste.braun.delvoye@gmail.com',
    description='The package to use the Dynamixel of the car for the direction',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_dir_node = bolide_direction.cmd_dir_node:main',
        ],
    },
)
