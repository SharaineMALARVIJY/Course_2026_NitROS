from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    bolide_teleop_dir = get_package_share_directory("bolide_teleop")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bolide_teleop_dir, "launch", "sensors.launch.py")
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        )
    )

    follow_gap_node = Node(
        package='bolide_wall_follow',
        executable='follow_gap',
        name='follow_gap',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        sensors_launch,

        TimerAction(
            period=2.0,
            actions=[slam_launch]
        ),

        TimerAction(
            period=6.0,
            actions=[follow_gap_node]
        ),
    ])
