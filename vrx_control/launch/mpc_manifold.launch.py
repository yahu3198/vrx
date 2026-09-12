"""MPC node for a deployment trial, with the trial cell as launch arguments.

ros2 launch vrx_control mpc_manifold.launch.py \
    degrade:=0.95 fault_type:=1 ref_source:=manifold trigger_x:=-459.5 \
    ref_traj:=/abs/path/approach_leg_0p8.txt
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("vrx_control")
    param_file = os.path.join(share, "config", "gazebo_param_manifold.yaml")
    default_traj = os.path.join(share, "traj", "approach_leg_0p8.txt")
    args = [
        DeclareLaunchArgument("degrade", default_value="0.95"),
        DeclareLaunchArgument("fault_type", default_value="1"),
        DeclareLaunchArgument("ref_source", default_value="manifold"),
        DeclareLaunchArgument("trigger_iters", default_value="3200"),
        DeclareLaunchArgument("trigger_x", default_value="-459.5"),
        DeclareLaunchArgument("timeout_s", default_value="60.0"),
        DeclareLaunchArgument("ref_traj", default_value=default_traj),
    ]
    node = Node(
        package="vrx_control", executable="wamv_mpc_node", name="wamv_mpc_node",
        output="screen",
        parameters=[param_file, {
            "thruster_degrade_percentage": LaunchConfiguration("degrade"),
            "fault_type_sim": LaunchConfiguration("fault_type"),
            "ref_source": LaunchConfiguration("ref_source"),
            "fault_trigger_iters": LaunchConfiguration("trigger_iters"),
            "use_position_trigger": True,
            "fault_trigger_x": LaunchConfiguration("trigger_x"),
            "manifold_timeout_s": LaunchConfiguration("timeout_s"),
            "ref_traj": LaunchConfiguration("ref_traj"),
            "arrival_strict": True,
        }],
    )
    return LaunchDescription(args + [node])
