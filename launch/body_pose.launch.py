"""Launch body_pose node with config; supports use_camera argument for camera or topic input."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("body_pose"), "config", "body_pose.yaml"]
    )
    use_camera_arg = DeclareLaunchArgument(
        "use_camera",
        default_value="true",
        description=(
            "Use camera (true) or subscribe to image_topic (false). "
            "Set false when using publish_video."
        ),
    )
    return LaunchDescription(
        [
            use_camera_arg,
            Node(
                package="body_pose",
                executable="body_pose_node",
                name="body_pose_node",
                output="screen",
                parameters=[
                    params_file,
                    {"use_camera": LaunchConfiguration("use_camera")},
                ],
            ),
        ]
    )
