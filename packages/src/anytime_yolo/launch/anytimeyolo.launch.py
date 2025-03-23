from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='models/',
        description='Path to the YOLO onnx folder'
    )
    
    anytime_yolo_node = Node(
        package='anytime_yolo',
        executable='anytime_yolo',
        name='anytime_yolo',
        output='screen',
    )
    
    return LaunchDescription([
        anytime_yolo_node
    ])