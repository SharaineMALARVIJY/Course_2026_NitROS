from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    map_file = LaunchConfiguration("map")
    amcl_params = LaunchConfiguration("amcl_params")
    nav2_params = LaunchConfiguration("nav2_params")
    use_sim_time = LaunchConfiguration("use_sim_time")

    bolide_teleop_dir = get_package_share_directory("bolide_teleop")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bolide_teleop_dir, "launch", "sensors.launch.py")
        )
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": use_sim_time,
            "params_file": amcl_params,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value="/home/voiture/bolide_voiture/maps/map_etage_2.yaml"
        ),
        DeclareLaunchArgument(
            "amcl_params",
            default_value="/home/voiture/bolide_voiture/config_nav2/amcl_params.yaml"
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value="/home/voiture/bolide_voiture/config_nav2/nav2_params.yaml"
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false"
        ),

        # 1) capteurs
        sensors_launch,

        # 2) localisation après un petit délai
        TimerAction(
            period=2.0,
            actions=[localization_launch]
        ),

        # 3) navigation après encore un petit délai
        TimerAction(
            period=5.0,
            actions=[navigation_launch]
        ),
    ])
