from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Publish static imu -> body transform for Unitree G1.

    The lidar/imu is 0.47618m above the body, rolled 180 deg (z-axis down,
    x-axis forward), then pitched 2 deg forward.
    body_T_imu: translation (0, 0, 0.47618), rotation = Ry(0.03491) * Rx(pi)
    imu_T_body (published here): inverse of the above.
    """
    # imu_T_body = Rx(-pi) * Ry(-0.03491) * T(0, 0, -0.47618)
    # translation: (0.01662, 0, 0.47589)
    # quaternion: Ry(0.03491) * Rx(pi) = (qx=0.99985, qy=0, qz=-0.01745, qw=0)
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_body_broadcaster',
            arguments=[
                '0.01662', '0', '0.47589',
                '0.99985', '0', '-0.01745', '0',
                'imu', 'body',
            ],
        ),
    ])
