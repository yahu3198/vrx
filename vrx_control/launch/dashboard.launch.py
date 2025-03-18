import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of vrx_control package
    vrx_control_dir = get_package_share_directory("vrx_control")

    # Full path to the parameter file
    param_file = os.path.join(vrx_control_dir, "config", "dashboard_param.yaml")

    # Create parameter dictionary in case the file doesn't exist
    dashboard_params = {
        "max_data_points": 200,
        "update_interval_ms": 100,
    }

    return LaunchDescription([
        # Launch the dashboard node
        Node(
            package="vrx_control",
            executable="wamv_dashboard_node",
            name="wamv_dashboard_node",
            output="screen",
            parameters=[dashboard_params],  # Use inline params if file doesn't exist
        )
    ])