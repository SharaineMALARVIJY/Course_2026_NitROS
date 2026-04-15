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
            {"angle_max_filter": 1.75},      # +100° (extrème droite)
            {"motor_speed": 750},

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
    cmd_twist_bridge_node = Node(
        package="bolide_stm32",
        executable="cmd_twist_bridge_node",
        name="cmd_twist_bridge_node",
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
                    {'max_steering_angle': 20.0},       # steering angle de chaque côté (en °) -ne correspond pas exactement à l'angle réel-
                    {'steering_offset_deg': -3.5}],     # offset (sens marche avant) : gauche >0, droit <0 (en°)
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
            "--roll", "0.0", # 3.14159265359
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "laser_frame",
        ]
    )

    tf_link_to_sonar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_link_to_laser",
        arguments=[
            "--x", "-0.20",
            "--y", "0",
            "--z", "0.10",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "3.14159",
            "--frame-id", "base_link",
            "--child-frame-id", "sonar_frame",
        ]
    )

    tf_link_to_ir_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_link_to_ir_left",
        arguments=[
            "--x", "-0.20",
            "--y", "0.05",
            "--z", "0.04",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "3.14159",
            "--frame-id", "base_link",
            "--child-frame-id", "ir_left_frame",
        ]
    )

    tf_link_to_ir_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_link_to_ir_right",
        arguments=[
            "--x", "-0.20",
            "--y", "-0.05",
            "--z", "0.04",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "3.14159",
            "--frame-id", "base_link",
            "--child-frame-id", "ir_right_frame",
        ]
    )

    return LaunchDescription(
        [
            odom_node,
            tf_base_to_link,
            tf_link_to_laser,
            # tf_link_to_sonar, #jamais tester
            tf_link_to_ir_left,
            tf_link_to_ir_right,
            sllidar,
            stm32_node,
            cmd_vel_node,
            cmd_dir_node,
            speed_controller,
            cmd_twist_bridge_node,
        ]
    )
