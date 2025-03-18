import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of vrx_control package
    vrx_control_dir = get_package_share_directory("vrx_control")

    # Include the MPC launch file
    mpc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vrx_control_dir, 'launch', 'mpc.launch.py')
        )
    )

    # Include the dashboard launch file
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vrx_control_dir, 'launch', 'dashboard.launch.py')
        )
    )

    # Return launch description
    return LaunchDescription([
        mpc_launch,
        dashboard_launch,
    ])