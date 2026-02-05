import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from datetime import datetime

def generate_launch_description():
    # Get package directory
    vrx_control_dir = get_package_share_directory("vrx_control")
    
    # Generate bag name with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = f"env_mpc_left_0.95_{timestamp}"
    bag_dir = os.path.expanduser("~/usv_ws/experiments/bags")
    
    # Create directory if it doesn't exist
    os.makedirs(bag_dir, exist_ok=True)
    
    # Topics to record
    topics = [
        '/wamv/sensors/position/ground_truth_odometry',
        '/wamv/thrusters/left/thrust',
        '/wamv/thrusters/right/thrust',
        '/wamv/ref_pose',
        '/wamv/error_pose',
        '/wamv/control_inputs',
        '/wamv/ekf_pose',
        '/wamv/disturbance',
        '/wamv/disturbance_world',
        '/wamv/fault_diagnosis',
        '/wamv/operational_mode',
        '/wamv/thruster_health',
        '/wamv/environmental_assistance',
        '/wamv/planning_status',
        '/wamv/prediction_metrics',
        '/wamv/mission_metrics',
        '/wamv/usv_state',
    ]
    
    # Start rosbag recording
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', f'{bag_dir}/{bag_name}'] + topics,
        output='screen',
        shell=False
    )
    
    # Include MPC launch file (with 2 second delay to ensure rosbag is ready)
    mpc_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(vrx_control_dir, 'launch', 'mpc.launch.py')
                )
            )
        ]
    )
    
    # Include dashboard launch file (also with delay)
    dashboard_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(vrx_control_dir, 'launch', 'dashboard.launch.py')
                )
            )
        ]
    )
    
    # Print info
    print(f"Recording to: {bag_dir}/{bag_name}")
    print("MPC and Dashboard will start in 2 seconds...")
    
    return LaunchDescription([
        rosbag_record,
        mpc_launch,
        dashboard_launch,
    ])