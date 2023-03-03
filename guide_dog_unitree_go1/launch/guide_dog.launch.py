from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_nav2',
            default_value='true',
            choices=['true','false'],
            description='Runs Nav2 for autonomous navigation'
        ),

        DeclareLaunchArgument(
            name='use_voice_control',
            default_value='true',
            choices=['true','false'],
            description='Enables voice control of unitree Go1'
        ),

        Node(
            package='guide_dog_unitree_go1',
            executable='voice_control',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_voice_control')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('unitree_nav'),
                    'launch',
                    'unitree_nav.launch.py'
                ])
            ),
            condition=IfCondition(LaunchConfiguration('use_nav2')),
        ),
    ])