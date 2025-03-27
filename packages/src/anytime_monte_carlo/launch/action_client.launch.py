from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Include launch description"""
    
    goal_timer_period = LaunchConfiguration('goal_timer_period_ms')
    cancel_timeout_period = LaunchConfiguration('cancel_timeout_period_ms')
    result_filename = LaunchConfiguration('result_filename')

    anytime_cmd = Node(
        package='anytime_monte_carlo',
        executable='anytime_action_client',
        name='anytime_client',
        output='screen',
        parameters=[{
            'goal_timer_period_ms': goal_timer_period,
            'cancel_timeout_period_ms': cancel_timeout_period,
            'result_filename': result_filename
        }]
    )

    cmds = []
    cmds.append(anytime_cmd)

    return cmds


def generate_launch_description():
    """Return launch description"""

    threading_type_arg = DeclareLaunchArgument(
        'threading_type',
        default_value='single',
        description='Threading type'
    )
    
    goal_timer_period_arg = DeclareLaunchArgument(
        'goal_timer_period_ms',
        default_value='100',
        description='Period in milliseconds for the goal request timer'
    )
    
    cancel_timeout_period_arg = DeclareLaunchArgument(
        'cancel_timeout_period_ms',
        default_value='50',
        description='Period in milliseconds for the cancel timeout timer'
    )

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='anytime_results',
        description='Filename for storing results (without extension)'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(threading_type_arg)
    launch_description.add_action(goal_timer_period_arg)
    launch_description.add_action(cancel_timeout_period_arg)
    launch_description.add_action(result_filename_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
