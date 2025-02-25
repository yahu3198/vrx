import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of vrx_control package
    vrx_control_dir = get_package_share_directory("vrx_control")

    # Full path to the parameter file
    param_file = os.path.join(vrx_control_dir, "config", "gazebo_param.yaml")

    # Check if the YAML file exists
    if not os.path.exists(param_file):
        raise FileNotFoundError(f"Parameter file not found: {param_file}")

    return LaunchDescription([
        Node(
            package="vrx_control",
            executable="wamv_mpc_node",
            name="wamv_mpc_node",
            output="screen",
            parameters=[param_file],  # Correct way to load params
        )
    ])
