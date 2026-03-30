import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _launch_hesai_driver(context):
    share_dir = get_package_share_directory('relocalization_bringup')
    resource_dir = os.path.join(share_dir, 'resource')
    config_path = os.path.join(share_dir, 'config', 'hesaiJT128_driver.yaml')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    lidar_udp = config['lidar'][0]['driver']['lidar_udp_type']
    lidar_udp['correction_file_path'] = os.path.join(resource_dir, 'JT128_default_angle.csv')
    lidar_udp['firetimes_path'] = os.path.join(resource_dir, 'JT128_Firetime_Correction.csv')

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='hesaiJT128_driver_'
    )
    yaml.dump(config, tmp)
    tmp.close()

    return [Node(
        namespace='hesai_ros_driver',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': tmp.name}]
    )]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_launch_hesai_driver),
    ])
