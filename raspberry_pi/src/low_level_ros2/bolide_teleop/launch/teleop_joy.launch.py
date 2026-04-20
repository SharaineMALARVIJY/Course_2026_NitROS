from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='bolide_teleop',
            executable='teleop_joy',
            name='teleop_joy',
            output='screen',
            parameters=[{
                'debug': True,
                'rate': 20.0,
                'deadzone': 0.08,
                'steering_scale': 0.7,
                'speed_scale': 1.0,
                'max_speed_forward': 1.5,
                'max_speed_reverse': 0.8,
                'accel_rate': 0.8,
                'decel_rate': 2.0,
                'enable_button_required': False,
                'speed_axis': 1,
                'direction_axis': 3,
                'enable_button': 5,
                'emergency_button': 4,
            }]
        )
    ])
