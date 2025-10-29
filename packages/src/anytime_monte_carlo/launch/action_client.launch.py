from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
import os
from ament_index_python.packages import get_package_share_directory


def include_launch_description(context: LaunchContext):
    """Include launch description"""

    config_file = LaunchConfiguration('config_file')
    goal_timer_period = LaunchConfiguration('goal_timer_period_ms')
    cancel_timeout_period = LaunchConfiguration('cancel_timeout_period_ms')
    log_level = LaunchConfiguration('log_level')

    # Get the config file path from context
    config_path = context.launch_configurations.get('config_file', '')

    print("\n" + "="*60)
    print("Monte Carlo Action Client Launch Configuration")
    print("="*60)

    # Prepare parameters - either from config file or command line arguments
    if config_path and os.path.exists(config_path):
        print(f"Loading config from: {config_path}")
        # Use config file
        parameters = [config_path]
        # Override with command line arguments if provided
        overrides = {}
        goal_timer_value = context.launch_configurations.get(
            'goal_timer_period_ms', '')
        cancel_timeout_value = context.launch_configurations.get(
            'cancel_timeout_period_ms', '')

        if goal_timer_value and goal_timer_value != '':
            overrides['goal_timer_period_ms'] = int(goal_timer_value)
            print(
                f"  [Override] goal_timer_period_ms: {goal_timer_value} ms (from command line)")
        if cancel_timeout_value and cancel_timeout_value != '':
            overrides['cancel_timeout_period_ms'] = int(cancel_timeout_value)
            print(
                f"  [Override] cancel_timeout_period_ms: {cancel_timeout_value} ms (from command line)")
        if overrides:
            parameters.append(overrides)
    else:
        print("Using command line arguments (no config file)")
        # Use command line arguments
        parameters = [{
            'goal_timer_period_ms': goal_timer_period,
            'cancel_timeout_period_ms': cancel_timeout_period
        }]

    # Get logger level from config or command line
    logger = context.launch_configurations.get('log_level', '')

    # If log_level not provided via command line, try to read from YAML config
    if (not logger or logger == '') and config_path and os.path.exists(config_path):
        import yaml
        try:
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                if config_data and 'anytime_client' in config_data:
                    params = config_data['anytime_client'].get(
                        'ros__parameters', {})
                    logger = params.get('log_level', 'info')
                    print(f"  log_level: {logger} (from config file)")
        except Exception as e:
            print(f"Warning: Could not read log_level from config file: {e}")
            logger = 'info'
            print(f"  log_level: {logger} (default)")
    elif logger and logger != '':
        print(f"  [Override] log_level: {logger} (from command line)")
    else:
        logger = 'info'
        print(f"  log_level: {logger} (default)")

    print("="*60 + "\n")

    # Build arguments list
    arguments = ['--ros-args', '--log-level', logger]

    anytime_cmd = Node(
        package='anytime_monte_carlo',
        executable='anytime_mc_client',
        name='anytime_client',
        output='screen',
        parameters=parameters,
        arguments=arguments
    )

    cmds = []
    cmds.append(anytime_cmd)

    return cmds


def generate_launch_description():
    """Return launch description"""

    # Get the package directory for anytime_monte_carlo
    try:
        package_dir = get_package_share_directory('anytime_monte_carlo')
        default_config_file = os.path.join(
            package_dir, 'config', 'client_params.yaml')
    except Exception as e:
        print(f"Warning: Could not get package directory: {e}")
        default_config_file = ""

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the client configuration YAML file'
    )

    goal_timer_period_arg = DeclareLaunchArgument(
        'goal_timer_period_ms',
        default_value='',
        description='Period in milliseconds for the goal request timer (overrides config file)'
    )

    cancel_timeout_period_arg = DeclareLaunchArgument(
        'cancel_timeout_period_ms',
        default_value='',
        description='Period in milliseconds for the cancel timeout timer (overrides config file)'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(config_file_arg)
    launch_description.add_action(goal_timer_period_arg)
    launch_description.add_action(cancel_timeout_period_arg)
    launch_description.add_action(log_level_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
