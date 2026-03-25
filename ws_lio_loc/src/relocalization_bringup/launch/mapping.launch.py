import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

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
    spark_fast_lio_path = get_package_share_directory('spark_fast_lio')

    default_config_path = os.path.join(bringup_path, 'config')
    default_rviz_config_path = os.path.join(
        spark_fast_lio_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='default',
        description='Robot name for body frame selection (default, g1, go2)'
    )

    spark_lio_node = Node(
        package='spark_fast_lio',
        executable='spark_lio_mapping',
        name='lio_mapping',
        remappings=[
            ('lidar', '/livox/lidar'),
            ('imu', '/livox/imu'),
        ],
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_robot_name_cmd)

    ld.add_action(spark_lio_node)
    ld.add_action(rviz_node)
    ld.add_action(OpaqueFunction(function=_include_body_frame))

    return ld
