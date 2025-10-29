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
    image_topic = LaunchConfiguration('image_topic')
    cancel_after_layers = LaunchConfiguration('cancel_after_layers')
    cancel_layer_score = LaunchConfiguration('cancel_layer_score')
    log_level = LaunchConfiguration('log_level')

    # Get the config file path from context
    config_path = context.launch_configurations.get('config_file', '')

    print("\n" + "="*60)
    print("YOLO Action Client Launch Configuration")
    print("="*60)

    # Prepare parameters - either from config file or command line arguments
    if config_path and os.path.exists(config_path):
        print(f"Loading config from: {config_path}")

        # Read config to show parameters
        import yaml
        try:
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                if config_data and 'anytime_client' in config_data:
                    params = config_data['anytime_client'].get(
                        'ros__parameters', {})
                    print(
                        f"  image_topic: {params.get('image_topic', 'N/A')} (from config file)")
                    print(
                        f"  goal_timer_period_ms: {params.get('goal_timer_period_ms', 'N/A')} (from config file)")
                    print(
                        f"  cancel_timeout_period_ms: {params.get('cancel_timeout_period_ms', 'N/A')} (from config file)")
                    print(
                        f"  cancel_after_layers: {params.get('cancel_after_layers', 'N/A')} (from config file)")
                    print(
                        f"  cancel_layer_score: {params.get('cancel_layer_score', 'N/A')} (from config file)")
        except Exception as e:
            print(f"Warning: Could not read parameters from config file: {e}")

        # Use config file
        parameters = [config_path]
        # Override with command line arguments if provided
        overrides = {}
        image_topic_value = context.launch_configurations.get(
            'image_topic', '')
        cancel_after_layers_value = context.launch_configurations.get(
            'cancel_after_layers', '')
        cancel_layer_score_value = context.launch_configurations.get(
            'cancel_layer_score', '')

        if image_topic_value and image_topic_value != '':
            overrides['image_topic'] = image_topic_value
            print(
                f"  [Override] image_topic: {image_topic_value} (from command line)")
        if cancel_after_layers_value and cancel_after_layers_value != '':
            overrides['cancel_after_layers'] = int(cancel_after_layers_value)
            print(
                f"  [Override] cancel_after_layers: {cancel_after_layers_value} (from command line)")
        if cancel_layer_score_value and cancel_layer_score_value != '':
            overrides['cancel_layer_score'] = cancel_layer_score_value.lower(
            ) == 'true'
            print(
                f"  [Override] cancel_layer_score: {cancel_layer_score_value} (from command line)")
        if overrides:
            parameters.append(overrides)
    else:
        print("Using command line arguments (no config file)")
        # Use command line arguments
        image_topic_value = context.launch_configurations.get(
            'image_topic', '')
        cancel_after_layers_value = context.launch_configurations.get(
            'cancel_after_layers', '')
        cancel_layer_score_value = context.launch_configurations.get(
            'cancel_layer_score', '')

        print(f"  image_topic: {image_topic_value} (from command line)")
        print(
            f"  cancel_after_layers: {cancel_after_layers_value} (from command line)")
        print(
            f"  cancel_layer_score: {cancel_layer_score_value} (from command line)")

        parameters = [{
            'image_topic': image_topic,
            'cancel_after_layers': cancel_after_layers,
            'cancel_layer_score': cancel_layer_score
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
    print("="*60 + "\n")

    # Build arguments list
    arguments = ['--ros-args', '--log-level', logger]

    anytime_cmd = Node(
        package='anytime_yolo',
        executable='anytime_yolo_client',
        name='anytime_client',
        parameters=parameters,
        arguments=arguments,
        # output='screen',
    )

    cmds = []
    cmds.append(anytime_cmd)

    return cmds


def generate_launch_description():
    """Return launch description"""

    # Get the package directory for anytime_yolo
    try:
        package_dir = get_package_share_directory('anytime_yolo')
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

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='',
        description='Topic name for the input image (overrides config file)'
    )

    cancel_after_layers_arg = DeclareLaunchArgument(
        'cancel_after_layers',
        default_value='',
        description='Number of processed layers before triggering cancellation (overrides config file)'
    )

    cancel_layer_score_arg = DeclareLaunchArgument(
        'cancel_layer_score',
        default_value='',
        description='Enable cancellation based on high score for id 9 in feedback (overrides config file)'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(config_file_arg)
    launch_description.add_action(image_topic_arg)
    launch_description.add_action(cancel_after_layers_arg)
    launch_description.add_action(cancel_layer_score_arg)
    launch_description.add_action(log_level_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
