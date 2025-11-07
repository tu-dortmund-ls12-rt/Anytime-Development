"""Launch file to start interference timer as a component."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    """Setup function to print executor type and return container."""

    use_multi_threaded = context.launch_configurations.get(
        'use_multi_threaded', 'false')
    container_name = context.launch_configurations.get(
        'container_name', 'interference_container')

    executor_type = "MULTI-THREADED" if use_multi_threaded == 'true' else "SINGLE-THREADED"
    executable_name = "component_container_mt" if use_multi_threaded == 'true' else "component_container"

    print("\n" + "="*80)
    print("Interference Component Container Launch")
    print("="*80)
    print(f"Container Name: {container_name}")
    print(f"Executor Type:  {executor_type}")
    print(f"Executable:     {executable_name}")
    print("="*80 + "\n")

    timer_period_ms = context.launch_configurations.get(
        'timer_period_ms', '100')
    execution_time_ms = context.launch_configurations.get(
        'execution_time_ms', '10')
    log_level = context.launch_configurations.get('log_level', 'debug')

    # Create component container with interference timer
    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable=executable_name,
        composable_node_descriptions=[
            ComposableNode(
                package='interference',
                plugin='interference::InterferenceTimerNode',
                name='interference_timer',
                parameters=[{
                    'timer_period_ms': int(timer_period_ms),
                    'execution_time_ms': int(execution_time_ms),
                }]
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    return [container]


def generate_launch_description():
    """Return launch description with component container."""

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

    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='interference_container',
        description='Name of the component container'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='debug',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    use_multi_threaded_arg = DeclareLaunchArgument(
        'use_multi_threaded',
        default_value='false',
        description='Use multi-threaded executor (true) or single-threaded (false)'
    )

    return LaunchDescription([
        timer_period_arg,
        execution_time_arg,
        container_name_arg,
        log_level_arg,
        use_multi_threaded_arg,
        OpaqueFunction(function=launch_setup)
    ])
