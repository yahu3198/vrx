"""
Launch file for Sensor Fault Injection Nodes (GPS + IMU)

Usage:
    # Launch all sensors
    ros2 launch sensor_fault_injection all_faults.launch.py

    # Launch with faults enabled
    ros2 launch sensor_fault_injection all_faults.launch.py \
        gps_fault_enabled:=true gps_fault_type:=4 \
        imu_fault_enabled:=true imu_fault_type:=2

    # Launch with degraded rate fault
    ros2 launch sensor_fault_injection all_faults.launch.py \
        gps_fault_enabled:=true gps_fault_type:=5 \
        imu_fault_enabled:=true imu_fault_type:=3

    # Launch only GPS
    ros2 launch sensor_fault_injection all_faults.launch.py enable_imu:=false

    # Launch only IMU
    ros2 launch sensor_fault_injection all_faults.launch.py enable_gps:=false

Fault Types:
    GPS: 0=None, 1=Outage, 2=Jump, 3=Stuck, 4=Multipath, 5=DegradedRate
    IMU: 0=None, 1=Stuck, 2=Saturation, 3=DegradedRate
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('sensor_fault_injection')
    
    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'fault_params.yaml')
    
    # =========================================================================
    # Declare Launch Arguments
    # =========================================================================
    
    # Config file
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration YAML file'
    )
    
    # Enable/disable individual sensors
    declare_enable_gps = DeclareLaunchArgument(
        'enable_gps',
        default_value='true',
        description='Enable GPS fault injection node'
    )
    
    declare_enable_imu = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='Enable IMU fault injection node'
    )
    
    # ----- GPS Arguments -----
    declare_gps_input_topic = DeclareLaunchArgument(
        'gps_input_topic',
        default_value='/wamv/sensors/gps/gps/fix',
        description='GPS input topic (clean data)'
    )
    
    declare_gps_output_topic = DeclareLaunchArgument(
        'gps_output_topic',
        default_value='/wamv/sensors/gps/gps/fix_faulty',
        description='GPS output topic (faulty data)'
    )
    
    declare_gps_fault_type = DeclareLaunchArgument(
        'gps_fault_type',
        default_value='0',
        description='GPS fault type: 0=None, 1=Outage, 2=Jump, 3=Stuck, 4=Multipath, 5=DegradedRate'
    )
    
    declare_gps_fault_enabled = DeclareLaunchArgument(
        'gps_fault_enabled',
        default_value='false',
        description='Enable GPS fault injection'
    )
    
    # ----- IMU Arguments -----
    declare_imu_input_topic = DeclareLaunchArgument(
        'imu_input_topic',
        default_value='/wamv/sensors/imu/imu/data',
        description='IMU input topic (clean data)'
    )
    
    declare_imu_output_topic = DeclareLaunchArgument(
        'imu_output_topic',
        default_value='/wamv/sensors/imu/imu/data_faulty',
        description='IMU output topic (faulty data)'
    )
    
    declare_imu_fault_type = DeclareLaunchArgument(
        'imu_fault_type',
        default_value='0',
        description='IMU fault type: 0=None, 1=Stuck, 2=Saturation, 3=DegradedRate'
    )
    
    declare_imu_fault_enabled = DeclareLaunchArgument(
        'imu_fault_enabled',
        default_value='false',
        description='Enable IMU fault injection'
    )
    
    # =========================================================================
    # Nodes
    # =========================================================================
    
    # GPS Fault Injection Node
    gps_fault_injection_node = Node(
        condition=IfCondition(LaunchConfiguration('enable_gps')),
        package='sensor_fault_injection',
        executable='gps_fault_injection_node.py',
        name='gps_fault_injection_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'input_topic': LaunchConfiguration('gps_input_topic'),
                'output_topic': LaunchConfiguration('gps_output_topic'),
                'fault_type': LaunchConfiguration('gps_fault_type'),
                'fault_enabled': LaunchConfiguration('gps_fault_enabled'),
            }
        ],
    )
    
    # IMU Fault Injection Node
    imu_fault_injection_node = Node(
        condition=IfCondition(LaunchConfiguration('enable_imu')),
        package='sensor_fault_injection',
        executable='imu_fault_injection_node.py',
        name='imu_fault_injection_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'input_topic': LaunchConfiguration('imu_input_topic'),
                'output_topic': LaunchConfiguration('imu_output_topic'),
                'fault_type': LaunchConfiguration('imu_fault_type'),
                'fault_enabled': LaunchConfiguration('imu_fault_enabled'),
            }
        ],
    )
    
    return LaunchDescription([
        # Config
        declare_config_file,
        
        # Enable/disable
        declare_enable_gps,
        declare_enable_imu,
        
        # GPS arguments
        declare_gps_input_topic,
        declare_gps_output_topic,
        declare_gps_fault_type,
        declare_gps_fault_enabled,
        
        # IMU arguments
        declare_imu_input_topic,
        declare_imu_output_topic,
        declare_imu_fault_type,
        declare_imu_fault_enabled,
        
        # Nodes
        gps_fault_injection_node,
        imu_fault_injection_node,
    ])