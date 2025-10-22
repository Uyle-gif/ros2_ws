#!/usr/bin/env python3
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_car_control')
    teleop_cfg   = join(pkg_share, 'config', 'teleop_twist_joy.yaml')
    mux_cfg      = join(pkg_share, 'config', 'twist_mux.yaml')
    stanley_cfg  = join(pkg_share, 'config', 'stanley_params.yaml')


    return LaunchDescription([
        # 1) Joystick driver -> /joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # 2) Teleop -> /cmd_vel/manual
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[teleop_cfg],
            remappings=[('/cmd_vel', '/cmd_vel/manual_raw')],
            output='screen'

            
        ),

        # 3) Stanley Controller -> /cmd_vel/auto
        Node(
            package='my_car_control',
            executable='stanley_controller',
            name='stanley_controller',
            parameters=[stanley_cfg],
            output='screen'
            
        ),

        # 4) Twist Mux -> /cmd_vel
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[mux_cfg],
            remappings=[('/cmd_vel_out', '/cmd_vel')],
            output='screen'
        ),

        Node(
            package='my_car_control',
            executable='smooth_cmd_vel',
            name='smooth_cmd_vel',
            output='screen'
        ),


    ])
