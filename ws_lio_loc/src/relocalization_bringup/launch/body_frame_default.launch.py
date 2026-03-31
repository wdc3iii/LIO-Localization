from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Publish static imu -> body transform (identity) for default robot."""
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_body_broadcaster',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'imu', 'body',
            ],
        ),
    ])
