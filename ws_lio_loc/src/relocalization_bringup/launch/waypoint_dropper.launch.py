import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ROBOT_BODY_FRAME_LAUNCH = {
    'default': 'body_frame_default.launch.py',
    'g1': 'body_frame_g1.launch.py',
    'go2': 'body_frame_go2.launch.py',
    'stick': 'body_frame_stick.launch.py',
}


def _include_body_frame(context):
    robot_name = context.launch_configurations['robot_name']
    bringup_path = get_package_share_directory('relocalization_bringup')

    launch_file = ROBOT_BODY_FRAME_LAUNCH.get(robot_name)
    if launch_file is None:
        raise RuntimeError(
            f"Unknown robot_name '{robot_name}'. "
            f"Valid options: {list(ROBOT_BODY_FRAME_LAUNCH.keys())}")

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, 'launch', launch_file)))]


def generate_launch_description():
    bringup_path = get_package_share_directory('relocalization_bringup')
    scan_lock_path = get_package_share_directory('scan_lock')

    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(bringup_path, 'config'),
        description='Config file directory'
    ))
    ld.add_action(DeclareLaunchArgument(
        'config_file', default_value='waypoint_dropper.yaml',
        description='Config file name'
    ))
    ld.add_action(DeclareLaunchArgument(
        'robot_name', default_value='default',
        description='Robot name for body frame selection (default, g1, go2, stick)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz_cfg',
        default_value=os.path.join(scan_lock_path, 'rviz', 'scanlock.rviz'),
        description='RViz config file'
    ))

    ld.add_action(Node(
        package='relocalization_bringup',
        executable='waypoint_dropper',
        name='waypoint_dropper',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
        ],
        output='screen',
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz),
    ))

    ld.add_action(OpaqueFunction(function=_include_body_frame))

    return ld
