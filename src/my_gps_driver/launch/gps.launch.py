from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            output='screen',
            parameters=['/home/uylegia/ros2_ws/src/my_gps_driver/config/neo_m8p.yaml']
        )
    ])
