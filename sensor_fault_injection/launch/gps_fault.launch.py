"""
Launch file for GPS Fault Injection Node

Usage:
    ros2 launch gps_fault_injection gps_fault_injection.launch.py

    # With custom parameters
    ros2 launch gps_fault_injection gps_fault_injection.launch.py fault_type:=1 fault_enabled:=true

    # With config file
    ros2 launch gps_fault_injection gps_fault_injection.launch.py config_file:=/path/to/config.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('sensor_fault_injection')
    
    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'fault_params.yaml')
    
    # Declare launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration YAML file'
    )
    
    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/wamv/sensors/gps/gps/fix',
        description='Input GPS topic (clean data)'
    )
    
    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/wamv/sensors/gps/gps/fix_faulty',
        description='Output GPS topic (faulty data)'
    )
    
    declare_fault_type = DeclareLaunchArgument(
        'fault_type',
        default_value='0',
        description='Fault type: 0=None, 1=Outage, 2=Jump, 3=Stuck, 4=Multipath'
    )
    
    declare_fault_enabled = DeclareLaunchArgument(
        'fault_enabled',
        default_value='false',
        description='Enable/disable fault injection'
    )
    
    # GPS Fault Injection Node
    gps_fault_injection_node = Node(
        package='sensor_fault_injection',
        executable='gps_fault_injection_node.py',
        name='gps_fault_injection_node',
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
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_input_topic,
        declare_output_topic,
        declare_fault_type,
        declare_fault_enabled,
        gps_fault_injection_node,
    ])