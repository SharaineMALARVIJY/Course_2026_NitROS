from setuptools import find_packages, setup

package_name = 'bolide_stm32'

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
    description='The package to use the STM32 and the sensors and motors connected to it',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_node = bolide_stm32.stm32_node:main',
            'cmd_twist_bridge_node = bolide_stm32.cmd_twist_bridge_node:main',
            'cmd_vel_node = bolide_stm32.cmd_vel_node:main',
            'esc_setup = bolide_stm32.esc_setup:main',
            'odom_node = bolide_stm32.odom_node:main',
            'speed_controller_node = bolide_stm32.speed_controller_node:main',
        ],
    },
)
