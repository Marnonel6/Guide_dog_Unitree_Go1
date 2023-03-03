"""Launch file for launching speech recognition and audio playback node"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='listen_talk_ros2',
            executable='listen',
            name='listen',
            output='screen',
        ),

        Node(
            package='listen_talk_ros2',
            executable='talk',
            name='talk',
            output='screen',
        )
    ])
