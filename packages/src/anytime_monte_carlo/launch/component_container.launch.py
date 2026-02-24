# Copyright 2025 Anytime System
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch file to start a component container with Monte Carlo client and server components."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Set up function to print executor type and return container."""
    use_multi_threaded = context.launch_configurations.get(
        'use_multi_threaded', 'false')
    container_name = context.launch_configurations.get(
        'container_name', 'monte_carlo_container')

    executor_type = (
        "MULTI-THREADED" if use_multi_threaded == 'true'
        else "SINGLE-THREADED"
    )
    executable_name = (
        "component_container_mt" if use_multi_threaded == 'true'
        else "component_container"
    )

    print("\n" + "="*80)
    print("Monte Carlo Component Container Launch")
    print("="*80)
    print(f"Container Name: {container_name}")
    print(f"Executor Type:  {executor_type}")
    print(f"Executable:     {executable_name}")
    print("="*80 + "\n")

    # Get the package directory
    package_dir = get_package_share_directory('anytime_monte_carlo')

    # Default config files
    default_client_config = os.path.join(
        package_dir, 'config', 'client_params.yaml')
    default_server_config = os.path.join(
        package_dir, 'config', 'server_params.yaml')

    client_config = context.launch_configurations.get(
        'client_config', default_client_config)
    server_config = context.launch_configurations.get(
        'server_config', default_server_config)
    log_level = context.launch_configurations.get('log_level', 'info')

    # Create component container with both client and server
    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable=executable_name,
        composable_node_descriptions=[
            ComposableNode(
                package='anytime_monte_carlo',
                plugin='AnytimeActionServer',
                name='anytime_server',
                parameters=[server_config],
            ),
            ComposableNode(
                package='anytime_monte_carlo',
                plugin='AnytimeActionClient',
                name='anytime_client',
                parameters=[client_config],
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    return [container]


def generate_launch_description():
    """Return launch description with component container."""
    # Get the package directory
    package_dir = get_package_share_directory('anytime_monte_carlo')

    # Default config files
    default_client_config = os.path.join(
        package_dir, 'config', 'client_params.yaml')
    default_server_config = os.path.join(
        package_dir, 'config', 'server_params.yaml')

    # Declare launch arguments
    client_config_arg = DeclareLaunchArgument(
        'client_config',
        default_value=default_client_config,
        description='Path to client configuration YAML file'
    )

    server_config_arg = DeclareLaunchArgument(
        'server_config',
        default_value=default_server_config,
        description='Path to server configuration YAML file'
    )

    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='monte_carlo_container',
        description='Name of the component container'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    use_multi_threaded_arg = DeclareLaunchArgument(
        'use_multi_threaded',
        default_value='false',
        description='Use multi-threaded executor (true) or single-threaded (false)'
    )

    return LaunchDescription([
        client_config_arg,
        server_config_arg,
        container_name_arg,
        log_level_arg,
        use_multi_threaded_arg,
        OpaqueFunction(function=launch_setup)
    ])
