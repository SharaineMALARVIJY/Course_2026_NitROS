from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown

restart_on_error = False

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
        respawn=restart_on_error,
        parameters=[{"debug": False},
                    {'cmd_vel_deadzone': 0.01},
                    {'pwm_forward_max': 2000},  # <= 2000 (ESC max)
                    {'pwm_forward_min': 1540},
                    {'pwm_reverse_min': 1460},
                    {'pwm_reverse_max': 1000}], # >= 1000 (ESC min)
    )
    cmd_vel_bridge_node = Node(
        package="bolide_stm32",
        executable="cmd_vel_bridge_node",
        name="cmd_vel_bridge_node",
        output="screen",
        respawn=restart_on_error,
        parameters=[{"debug": False}],
    )
    cmd_dir_node = Node(
        package="bolide_direction",
        executable="cmd_dir_node",
        name="cmd_dir_node",
        output="screen",
        respawn=restart_on_error,
        parameters=[{"debug": False},
                    {'baudrate': 1000000},
                    {'max_steering_angle': 21.0},       # steering angle de chaque côté (en °) -ne correspond pas exactement à l'angle réel-
                    {'steering_offset_deg': -1.5}],     # offset (sens marche avant) : gauche >0, droit <0 (en°)
    )
    odom_node = Node(
        package="bolide_stm32",
        executable="odom_node",
        name="odom_node",
        output="screen",
    )
    tf_base_to_link = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_link",
    arguments=[
        "--x", "0",
        "--y", "0",
        "--z", "0",
        "--roll", "0",
        "--pitch", "0",
        "--yaw", "0",
        "--frame-id", "base_footprint",
        "--child-frame-id", "base_link",
        ]
    )

    tf_link_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_link_to_laser",
        arguments=[
            "--x", "0.18",
            "--y", "0",
            "--z", "0.10",
            "--roll", "3.14159265359",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "laser_frame",
        ]
    )
    teleop_keyboard = Node(
        package="bolide_teleop",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        output="screen",
        respawn=restart_on_error,
        parameters=[
            {'debug': False},
            {'speed_increment': 0.01},
            {'direction_increment': 0.2},
            {'rate': 10}                        # Hz
        ]
    )


    # Quand teleop_keyboard se termine → shutdown de tout le launch
    shutdown_on_teleop_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=teleop_keyboard,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )


    return LaunchDescription(
        [
            odom_node,
            tf_base_to_link,
            tf_link_to_laser,
            sllidar,
            stm32_node,
            cmd_vel_node,
            cmd_dir_node,
            cmd_vel_bridge_node,
            #teleop_keyboard,
            #shutdown_on_teleop_exit,
        ]
    )
