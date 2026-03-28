from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Publish static imu -> body transform for stick-mounted lidar.

    The lidar/imu is 1m above and 30 deg pitched forward from the body frame.
    body_T_imu: translation (0, 0, 1), pitch = 0.5236 rad (30 deg)
    imu_T_body (published here): inverse of the above.
    """
    # imu_T_body = inverse of body_T_imu where body_T_imu = T(0,0,1) * Ry(30deg)
    # translation: Ry(-30deg) * (0, 0, -1) = (sin(30), 0, -cos(30)) = (0.5, 0, -0.866)
    # quaternion: Ry(-30deg) = (qx=0, qy=-0.25882, qz=0, qw=0.96593)
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_body_broadcaster',
            arguments=[
                '--x', '0.5', '--y', '0', '--z', '-0.866',
                '--qx', '0', '--qy', '-0.25882', '--qz', '0', '--qw', '0.96593',
                '--frame-id', 'imu',
                '--child-frame-id', 'body',
            ],
        ),
    ])
    # return LaunchDescription([
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='imu_to_body_broadcaster',
    #         arguments=[
    #             '--x', '0.5', '--y', '0', '--z', '-0.866',
    #             '--qx', '-0.18301', '--qy', '-0.18301', '--qz', '0.68301', '--qw', '0.68301',
    #             '--frame-id', 'imu',
    #             '--child-frame-id', 'body',
    #         ],
    #     ),
    # ])
