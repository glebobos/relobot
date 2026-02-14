from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate a launch description for the camera node with AprilTag detection.

    Returns
    -------
        LaunchDescription: the launch description
    """
    # Parameters for camera
    camera_param_name = "camera"
    camera_param_default = "0"
    camera_param = LaunchConfiguration(camera_param_name, default=camera_param_default)
    camera_launch_arg = DeclareLaunchArgument(
        camera_param_name,
        default_value=camera_param_default,
        description="camera ID or name"
    )

    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Camera Node
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='camera',
                parameters=[{
                    "camera": camera_param,
                    "width": 800,
                    "height": 600,
                    "format": "UYVY",
                    "FrameDurationLimits": [100000, 100000],
                    "camera_info_url": "file:///ros2_ws/src/camera_ros/calibration/camera.yaml"
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # AprilTag Node
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                parameters=[os.path.join(
                    get_package_share_directory('camera_ros'),
                    'config',
                    'apriltag.yaml'
                )],
                remappings=[
                    ("image_rect", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        camera_launch_arg,
        container,
    ])
