"""
Launch file for IMU Fault Injection Node only

Usage:
    ros2 launch sensor_fault_injection imu_fault.launch.py
    ros2 launch sensor_fault_injection imu_fault.launch.py fault_enabled:=true fault_type:=2

    # With degraded rate fault
    ros2 launch sensor_fault_injection imu_fault.launch.py fault_enabled:=true fault_type:=3

Fault Types:
    0 = None (pass-through)
    1 = Stuck (frozen readings)
    2 = Saturation (clipped values)
    3 = Degraded Rate (reduced frequency)
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
        default_value='/wamv/sensors/imu/imu/data',
        description='Input IMU topic'
    )
    
    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/wamv/sensors/imu/imu/data_faulty',
        description='Output IMU topic'
    )
    
    declare_fault_type = DeclareLaunchArgument(
        'fault_type',
        default_value='0',
        description='Fault type: 0=None, 1=Stuck, 2=Saturation, 3=DegradedRate'
    )
    
    declare_fault_enabled = DeclareLaunchArgument(
        'fault_enabled',
        default_value='false',
        description='Enable fault injection'
    )
    
    imu_fault_injection_node = Node(
        package='sensor_fault_injection',
        executable='imu_fault_injection_node.py',
        name='imu_fault_injection_node',
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
        imu_fault_injection_node,
    ])