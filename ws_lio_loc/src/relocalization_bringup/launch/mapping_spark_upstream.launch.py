import os.path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    default_rviz_config_path = '/home/wcompton/repos/LIO-Localization/ws_lio_loc/src/spark-fast-lio/spark_fork/rviz/fastlio.rviz'

    config_path = '/home/wcompton/repos/LIO-Localization/ws_lio_loc/src/relocalization_bringup/config'
    config_file = 'mid360_spark_upstream.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    spark_lio_node = Node(
        package='spark_fast_lio',
        executable='spark_lio_mapping',
        name='lio_mapping',
        remappings=[
            ('lidar', '/livox/lidar'),
            ('imu', '/livox/imu'),
        ],
        parameters=[os.path.join(config_path, config_file),
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
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(spark_lio_node)
    ld.add_action(rviz_node)

    return ld
