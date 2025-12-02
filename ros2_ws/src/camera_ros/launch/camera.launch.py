from launch.actions import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description for the camera node.

    Returns
    -------
        LaunchDescription: the launch description

    """
    # parameters
    camera_param_name = "camera"
    camera_param_default = str(0)
    camera_param = LaunchConfiguration(
        camera_param_name,
        default=camera_param_default,
    )
    camera_launch_arg = DeclareLaunchArgument(
        camera_param_name,
        default_value=camera_param_default,
        description="camera ID or name"
    )

    format_param_name = "format"
    format_param_default = str()
    format_param = LaunchConfiguration(
        format_param_name,
        default=format_param_default,
    )
    format_launch_arg = DeclareLaunchArgument(
        format_param_name,
        default_value=format_param_default,
        description="pixel format"
    )

    parameter_update_interval_name = "parameter_update_interval"
    parameter_update_interval_default = "30"
    parameter_update_interval_param = LaunchConfiguration(
        parameter_update_interval_name,
        default=parameter_update_interval_default,
    )
    parameter_update_interval_arg = DeclareLaunchArgument(
        parameter_update_interval_name,
        default_value=parameter_update_interval_default,
        description="Interval (in frames) for checking parameter updates"
    )

    # Standalone camera node - lightweight, no container overhead
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        parameters=[{
            "camera": camera_param,
            "width": 640,
            "height": 480,
            "format": "YUYV",
            "jpeg_quality": 50,
            "parameter_update_interval": parameter_update_interval_param,
        }],
        output='screen',
    )

    return LaunchDescription([
        camera_launch_arg,
        format_launch_arg,
        parameter_update_interval_arg,
        camera_node,
    ])
