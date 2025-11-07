#!/usr/bin/env python3
"""Launch file for interference timer node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for interference timer."""

    # Declare launch arguments
    timer_period_arg = DeclareLaunchArgument(
        'timer_period_ms',
        default_value='100',
        description='Timer period in milliseconds'
    )

    execution_time_arg = DeclareLaunchArgument(
        'execution_time_ms',
        default_value='10',
        description='Execution time (busy-wait duration) in milliseconds'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal)'
    )

    # Create interference timer node
    interference_timer_node = Node(
        package='interference',
        executable='interference_timer_node',
        name='interference_timer',
        output='screen',
        parameters=[{
            'timer_period_ms': LaunchConfiguration('timer_period_ms'),
            'execution_time_ms': LaunchConfiguration('execution_time_ms'),
        }],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        timer_period_arg,
        execution_time_arg,
        log_level_arg,
        interference_timer_node,
    ])
