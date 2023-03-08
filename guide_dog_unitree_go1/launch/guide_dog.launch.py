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

        DeclareLaunchArgument(
            name='use_speech_recognition',
            default_value='false',
            choices=['true','false'],
            description='Enables voice to text commands and audio playback from unitree Go1'
        ),

        DeclareLaunchArgument(
            name='use_object_detection',
            default_value='false',
            choices=['true','false'],
            description='Enables object detection with YOLOv7 on a realsense'
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
                    FindPackageShare('listen_talk_ros2'),
                    'listen_talk.launch.py'
                ])
            ),
            condition=IfCondition(LaunchConfiguration('use_speech_recognition')),
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('guide_dog_unitree_go1'),
                    'launch',
                    'vision.launch.xml'
                ])
            ),
            condition=IfCondition(LaunchConfiguration('use_object_detection')),
        ),
    ])