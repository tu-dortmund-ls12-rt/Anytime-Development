from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Include launch description"""

    is_reactive_proactive = LaunchConfiguration("is_reactive_proactive")
    batch_size = LaunchConfiguration("batch_size")

    # Determine the threading mode from launch context
    if context.launch_configurations["multi_threading"].lower() == "false":
        is_single_multi = "single"
    elif context.launch_configurations["multi_threading"].lower() == "true":
        is_single_multi = "multi"
    else:
        raise ValueError("Invalid threading type")

    anytime_cmd = Node(
        package="anytime_monte_carlo",
        executable="anytime_action_server",
        name="anytime_server",
        parameters=[{
            "is_reactive_proactive": is_reactive_proactive,
            "batch_size": batch_size,
            "is_single_multi": is_single_multi,
        }],
        arguments=["--is_single_multi", is_single_multi]
    )

    cmds = []
    cmds.append(anytime_cmd)

    return cmds


def generate_launch_description():
    """Return launch description"""

    threading_type_arg = DeclareLaunchArgument(
        "multi_threading", default_value="False", description="Threading type"
    )

    anytime_reactive_proactive_arg = DeclareLaunchArgument(
        "is_reactive_proactive", default_value="reactive", description="Anytime reactive"
    )

    batch_size_arg = DeclareLaunchArgument(
        "batch_size", default_value="2", description="Batch size for compute iterations"
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(threading_type_arg)
    launch_description.add_action(anytime_reactive_proactive_arg)
    launch_description.add_action(batch_size_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
