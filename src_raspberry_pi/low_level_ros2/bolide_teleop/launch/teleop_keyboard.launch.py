from launch import LaunchDescription
from launch_ros.actions import Node

restart_on_error = True

def generate_launch_description():
    sllidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        respawn=restart_on_error,
        parameters=[
            {"serial_port": "/dev/ttyLIDAR"},
            {"serial_baudrate": 256000},
            {"frame_id": "laser_frame"},
            {"inverted": False},
            {"angle_compensate": True},
            {"scan_mode": "Express"},
        ]
    )
    stm32_node = Node(
        package="bolide_stm32",
        executable="stm32_node",
        name="stm32_node",
        output="screen",
        respawn=restart_on_error,
        parameters=[{"debug": False}],
    )
    cmd_vel_node = Node(
        package="bolide_stm32",
        executable="cmd_vel_node",
        name="cmd_vel_node",
        output="screen",
        parameters=[{"debug": False}],
        respawn=restart_on_error,
    )
    cmd_dir_node = Node(
        package="bolide_direction",
        executable="cmd_dir_node",
        name="cmd_dir_node",
        output="screen",
        respawn=restart_on_error,
        parameters=[{"debug": False}],
    )
    teleop_keyboard = Node(
        package="bolide_teleop",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        output="screen",
        respawn=restart_on_error,
        parameters=[
            {'debug': False},
            {'speed_increment': 0.04},
            {'direction_increment': 0.2},
            {'rate': 10}                        # Hz
        ]
    )

    return LaunchDescription(
        [
            sllidar,
            stm32_node,
            cmd_vel_node,
            cmd_dir_node,
            teleop_keyboard,
        ]
    )
