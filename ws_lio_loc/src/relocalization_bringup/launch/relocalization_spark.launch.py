import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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
        return []

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, 'launch', launch_file)))]


def generate_launch_description():
    bringup_path = get_package_share_directory('relocalization_bringup')
    scan_lock_path = get_package_share_directory('scan_lock')

    # Shared arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # spark_fast_lio arguments
    lio_config_path = LaunchConfiguration('lio_config_path')
    lio_config_file = LaunchConfiguration('lio_config_file')
    rviz = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    # scan_lock arguments
    scan_lock_config_path = LaunchConfiguration('scan_lock_config_path')
    scan_lock_config_file = LaunchConfiguration('scan_lock_config_file')

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ))
    ld.add_action(DeclareLaunchArgument(
        'lio_config_path',
        default_value=os.path.join(bringup_path, 'config'),
        description='spark_fast_lio config file path'
    ))
    ld.add_action(DeclareLaunchArgument(
        'lio_config_file', default_value='mid360_relocalization_spark.yaml',
        description='spark_fast_lio config file'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz_cfg',
        default_value=os.path.join(
            get_package_share_directory('scan_lock'), 'rviz', 'scanlock.rviz'),
        description='RViz config file path'
    ))
    ld.add_action(DeclareLaunchArgument(
        'robot_name', default_value='none',
        description='Robot name for body frame selection (default, g1, go2, stick). If unset, no body frame TF is launched.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'scan_lock_config_path',
        default_value=os.path.join(bringup_path, 'config'),
        description='scan_lock config file path'
    ))
    ld.add_action(DeclareLaunchArgument(
        'scan_lock_config_file', default_value='scan_lock_spark.yaml',
        description='scan_lock config file'
    ))

    # Launch spark_fast_lio via mapping_spark.launch.py
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, 'launch', 'mapping_spark.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_path': lio_config_path,
            'config_file': lio_config_file,
            'rviz': rviz,
            'rviz_cfg': rviz_cfg,
        }.items()
    ))

    # Launch body frame TF broadcaster
    ld.add_action(OpaqueFunction(function=_include_body_frame))

    # Launch scan_lock via scan_lock.launch.py
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_lock_path, 'launch', 'scan_lock.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_path': scan_lock_config_path,
            'config_file': scan_lock_config_file,
        }.items()
    ))

    return ld
