from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate a launch description for the camera node with AprilTag detection
    and dock pose publisher bridge.

    AprilTag is NOT loaded at startup — the apriltag_manager node will
    dynamically load it into camera_container only when a dock goal is active,
    saving ~30 % CPU when idle.

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
            # AprilTag is loaded on-demand by apriltag_manager
            # Web Video Server for HTTP streaming (port 8080)
            ComposableNode(
                package='web_video_server',
                plugin='web_video_server::WebVideoServer',
                name='web_video_server',
                parameters=[{
                    'port': 8080,
                    'server_threads': 1,
                    'ros_threads': 2,
                    'default_stream_type': 'mjpeg',
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    always_on_env = os.environ.get('ALWAYS_ON_APRILTAG', 'false').lower() == 'true'

    # AprilTag manager: dynamically loads/unloads AprilTag + dock_pose_publisher
    # when the /dock_robot action becomes active/inactive.
    apriltag_manager = Node(
        package='camera_ros',
        executable='apriltag_manager',
        name='apriltag_manager',
        output='screen',
        parameters=[{'always_on': always_on_env}],
    )

    return LaunchDescription([
        camera_launch_arg,
        container,
        apriltag_manager,
    ])


