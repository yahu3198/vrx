"""
Launch file for LiDAR Fault Injection Node

Usage:
    ros2 launch sensor_fault_injection lidar_fault.launch.py
    ros2 launch sensor_fault_injection lidar_fault.launch.py fault_enabled:=true fault_type:=1

    # With increased noise fault
    ros2 launch sensor_fault_injection lidar_fault.launch.py fault_enabled:=true fault_type:=1

    # With reduced range fault
    ros2 launch sensor_fault_injection lidar_fault.launch.py fault_enabled:=true fault_type:=2

    # With stuck fault
    ros2 launch sensor_fault_injection lidar_fault.launch.py fault_enabled:=true fault_type:=3

Fault Types:
    0 = None (pass-through)
    1 = Increased Noise (rain, fog, spray, dust)
    2 = Reduced Range (fog, dirty lens)
    3 = Stuck (frozen point cloud)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_fault_injection')
    default_config = os.path.join(pkg_share, 'config', 'fault_params.yaml')
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration YAML file'
    )
    
    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/wamv/sensors/lidars/lidar_wamv_sensor/points',
        description='Input LiDAR point cloud topic'
    )
    
    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/wamv/sensors/lidars/lidar_wamv_sensor/points_faulty',
        description='Output LiDAR point cloud topic'
    )
    
    declare_fault_type = DeclareLaunchArgument(
        'fault_type',
        default_value='0',
        description='Fault type: 0=None, 1=IncreasedNoise, 2=ReducedRange, 3=Stuck'
    )
    
    declare_fault_enabled = DeclareLaunchArgument(
        'fault_enabled',
        default_value='false',
        description='Enable fault injection'
    )
    
    lidar_fault_injection_node = Node(
        package='sensor_fault_injection',
        executable='lidar_fault_injection_node.py',
        name='lidar_fault_injection_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'fault_type': LaunchConfiguration('fault_type'),
                'fault_enabled': LaunchConfiguration('fault_enabled'),
            }
        ],
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_input_topic,
        declare_output_topic,
        declare_fault_type,
        declare_fault_enabled,
        lidar_fault_injection_node,
    ])
