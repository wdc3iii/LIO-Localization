import os.path
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _load_params(yaml_path):
    """Load a ROS2 YAML parameter file and flatten to dot-separated keys."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    # Strip /**:/ros__parameters or <node_name>:/ros__parameters
    params = {}
    for key in data:
        if 'ros__parameters' in data[key]:
            params = data[key]['ros__parameters']
            break

    def _flatten(d, prefix=''):
        flat = {}
        for k, v in d.items():
            full_key = f'{prefix}{k}' if not prefix else f'{prefix}.{k}'
            if isinstance(v, dict):
                flat.update(_flatten(v, full_key))
            else:
                flat[full_key] = v
        return flat

    return _flatten(params)

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


def _launch_setup(context):
    """Resolve paths and load YAML parameters for ComposableNodes.
    Foxy's ComposableNode doesn't reliably load YAML file paths
    passed in the parameters list, so we parse them in Python."""
    use_sim_time = context.launch_configurations['use_sim_time']
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    lio_yaml = os.path.join(
        context.launch_configurations['lio_config_path'],
        context.launch_configurations['lio_config_file'])
    scan_lock_yaml = os.path.join(
        context.launch_configurations['scan_lock_config_path'],
        context.launch_configurations['scan_lock_config_file'])

    lio_params = _load_params(lio_yaml)
    lio_params['use_sim_time'] = use_sim_time == 'true'

    sl_params = _load_params(scan_lock_yaml)
    sl_params['use_sim_time'] = use_sim_time == 'true'

    # Composable node container with intra-process communication.
    # This avoids DDS inter-process transport for shared topics
    # (cloud_registered), which causes odometry degradation on
    # resource-constrained hardware (Jetson) when multiple
    # inter-process subscribers exist.
    container = ComposableNodeContainer(
        name='relocalization_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='spark_fast_lio',
                plugin='spark_fast_lio::SPARKFastLIO2',
                name='lio_mapping',
                remappings=[
                    ('lidar', '/livox/lidar'),
                    ('imu', '/livox/imu'),
                ],
                parameters=[lio_params],
                extra_arguments=[
                    {'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='scan_lock',
                plugin='scan_lock::ScanLockNode',
                name='scan_lock',
                parameters=[sl_params],
                extra_arguments=[
                    {'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    return [container, rviz_node]


def generate_launch_description():
    bringup_path = get_package_share_directory('relocalization_bringup')

    default_lio_config_path = os.path.join(bringup_path, 'config')
    default_rviz_config_path = os.path.join(
        get_package_share_directory('scan_lock'), 'rviz', 'scanlock.rviz')

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ))
    ld.add_action(DeclareLaunchArgument(
        'lio_config_path',
        default_value=default_lio_config_path,
        description='spark_fast_lio config file path'
    ))
    ld.add_action(DeclareLaunchArgument(
        'lio_config_file', default_value='mid360.yaml',
        description='spark_fast_lio config file'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    ))
    ld.add_action(DeclareLaunchArgument(
        'robot_name', default_value='default',
        description='Robot name for body frame selection (default, g1, go2)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'scan_lock_config_path',
        default_value=os.path.join(bringup_path, 'config'),
        description='scan_lock config file path'
    ))
    ld.add_action(DeclareLaunchArgument(
        'scan_lock_config_file', default_value='scan_lock.yaml',
        description='scan_lock config file'
    ))

    ld.add_action(OpaqueFunction(function=_launch_setup))
    ld.add_action(OpaqueFunction(function=_include_body_frame))

    return ld
