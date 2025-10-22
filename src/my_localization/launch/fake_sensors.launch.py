from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='my_localization', executable='fake_imu', name='fake_imu', output='screen'),
        Node(package='my_localization', executable='fake_gps', name='fake_gps', output='screen'),
    ])

