from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler,EmitEvent, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # sensors_launch
    ## 1. Localiser le chemin du fichier à inclure
    second_launch_path = os.path.join(
        get_package_share_directory('bolide_teleop'),
        'launch',
        'sensors.launch.py'
    )

    ## 2. Créer l'action d'inclusion
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(second_launch_path),
        #launch_arguments={'mon_argument': 'valeur'}.items() # Optionnel : passer des arguments
    )

    # Noeuds
    teleop_keyboard = Node(
        package="bolide_teleop",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        output="screen",
        parameters=[
            {'debug': False},
            {'speed_increment': 1.0/30.0},
            {'direction_increment': 0.2},
            {'rate': 10}                        # Hz
        ]
    )
    ## Quand teleop_keyboard se termine → shutdown de tout le launch
    shutdown_on_teleop_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=teleop_keyboard,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )
    return LaunchDescription([
        included_launch,
        teleop_keyboard,
        shutdown_on_teleop_exit,
    ])