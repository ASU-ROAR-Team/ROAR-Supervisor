#launch/multi_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='pub_sub', executable='talker', name='talker_1'),
        Node(package='pub_sub', executable='talker', name='talker_2'),
        Node(package='pub_sub', executable='listener', name='listener_1'),
        Node(package='pub_sub', executable='listener', name='listener_2'),
    ])