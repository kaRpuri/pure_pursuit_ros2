"""Launch the pure pursuit node with either the simulator or the car config."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')

    config_file = PathJoinSubstitution([
        FindPackageShare('pure_pursuit'),
        'config',
        ['pure_pursuit_', mode, '.yaml'],
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            choices=['sim', 'car'],
            description='sim loads pure_pursuit_sim.yaml, car loads pure_pursuit_car.yaml',
        ),
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
