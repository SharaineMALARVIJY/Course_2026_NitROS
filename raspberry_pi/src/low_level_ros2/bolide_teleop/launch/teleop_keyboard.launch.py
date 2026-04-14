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
            {"angle_min_filter": -1.75},    # -100° (extrème gauche)
            {"angle_max_filter": 1.75}      # +100° (extrème droite)
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
        parameters=[{"debug": False}]
    )
    speed_controller = Node(
    package='bolide_stm32',
    executable='speed_controller_node',
    parameters=[{
        'debug': True,
        'debug_freq': 2.0,
        'max_speed_forward': 3.0,
        'max_speed_reverse': 1.5
    }]
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
                    {'steering_offset_deg': -1.2}],     # offset (sens marche avant) : gauche >0, droit <0 (en°)
    )
    teleop_keyboard = Node(
        package="bolide_teleop",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        output="screen",
        respawn=restart_on_error,
        parameters=[
            {'debug': False},
            {'speed_increment': 1.0/30.0},
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
            sllidar,
            stm32_node,
            cmd_vel_node,
            speed_controller,
            cmd_dir_node,
            teleop_keyboard,
            shutdown_on_teleop_exit,
        ]
    )
