from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Publish static imu -> body transform for Unitree Go2.

    The lidar/imu is 0.25m above the body, rolled 180 deg (z-axis down,
    x-axis forward relative to body).
    body_T_imu: translation (0, 0, 0.25), roll = pi
    imu_T_body (published here): inverse of the above.
    """
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_body_broadcaster',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.25',
                '--qx', '1', '--qy', '0', '--qz', '0', '--qw', '0',
                '--frame-id', 'imu',
                '--child-frame-id', 'body',
            ],
        ),
    ])
