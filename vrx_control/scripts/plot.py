import matplotlib.pyplot as plt
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from transforms3d.euler import quat2euler
import rclpy
from rclpy.serialization import deserialize_message

def read_bag_data(bag_dir):
    # Initialize data storage
    error_pose_x = []
    error_pose_y = []
    error_pose_yaw = []
    error_pose_time = []

    ref_pose_x = []
    ref_pose_y = []
    ref_pose_yaw = []
    ref_pose_time = []

    pose_gt_x = []
    pose_gt_y = []
    pose_gt_yaw = []
    pose_gt_time = []

    ekf_pose_x = []
    ekf_pose_y = []
    ekf_pose_yaw = []
    ekf_pose_time = []

    left_thrust_angle = []
    right_thrust_angle = []
    left_thrust_cmd = []
    right_thrust_cmd = []
    control_inputs_time = []

    # Setup ROS 2 bag reader
    storage_options = StorageOptions(
        uri=bag_dir,
        storage_id='sqlite3'
    )
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Read all messages
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        timestamp_sec = timestamp * 1e-9  # Convert nanoseconds to seconds

        if topic == '/wamv/error_pose':
            msg = deserialize_message(data, Odometry)
            error_pose_x.append(msg.pose.pose.position.x)
            error_pose_y.append(msg.pose.pose.position.y)
            orientation_q = msg.pose.pose.orientation
            yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])[2]
            error_pose_yaw.append(yaw)
            error_pose_time.append(timestamp_sec)

        elif topic == '/wamv/ref_pose':
            msg = deserialize_message(data, Odometry)
            ref_pose_x.append(msg.pose.pose.position.x)
            ref_pose_y.append(msg.pose.pose.position.y)
            orientation_q = msg.pose.pose.orientation
            yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])[2]
            ref_pose_yaw.append(yaw)
            ref_pose_time.append(timestamp_sec)

        elif topic == '/wamv/sensors/position/ground_truth_odometry':
            msg = deserialize_message(data, Odometry)
            pose_gt_x.append(msg.pose.pose.position.x)
            pose_gt_y.append(msg.pose.pose.position.y)
            orientation_q = msg.pose.pose.orientation
            yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])[2]
            pose_gt_yaw.append(yaw)
            pose_gt_time.append(timestamp_sec)

        elif topic == '/wamv/control_inputs':
            msg = deserialize_message(data, TwistStamped)
            left_thrust_angle.append(msg.twist.linear.x)
            left_thrust_cmd.append(msg.twist.linear.y)
            right_thrust_angle.append(msg.twist.angular.x)
            right_thrust_cmd.append(msg.twist.angular.y)
            control_inputs_time.append(timestamp_sec)

        elif topic == '/wamv/ekf_pose':
            msg = deserialize_message(data, Odometry)
            ekf_pose_x.append(msg.pose.pose.position.x)
            ekf_pose_y.append(msg.pose.pose.position.y)
            orientation_q = msg.pose.pose.orientation
            yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])[2]
            ekf_pose_yaw.append(yaw)
            ekf_pose_time.append(timestamp_sec)

    return (error_pose_x, error_pose_y, error_pose_yaw, error_pose_time,
            ref_pose_x, ref_pose_y, ref_pose_yaw, ref_pose_time,
            pose_gt_x, pose_gt_y, pose_gt_yaw, pose_gt_time,
            ekf_pose_x, ekf_pose_y, ekf_pose_yaw, ekf_pose_time,
            left_thrust_angle, right_thrust_angle, left_thrust_cmd, right_thrust_cmd, control_inputs_time)

def plot_data(error_pose_x, error_pose_y, error_pose_yaw, error_pose_time,
              ref_pose_x, ref_pose_y, ref_pose_yaw, ref_pose_time,
              pose_gt_x, pose_gt_y, pose_gt_yaw, pose_gt_time,
              ekf_pose_x, ekf_pose_y, ekf_pose_yaw, ekf_pose_time,
              left_thrust_angle, right_thrust_angle, left_thrust_cmd, right_thrust_cmd, control_inputs_time):
    # Plot Figure 1: Error States
    fig, axs = plt.subplots(3, 1, figsize=(10, 10))
    fig.suptitle('Error States')

    axs[0].plot(error_pose_time, error_pose_x, label="X Error")
    axs[0].axhline(0, color='black', linestyle='--')
    axs[0].set_ylabel("X Error")
    axs[0].legend()

    axs[1].plot(error_pose_time, error_pose_y, label="Y Error")
    axs[1].axhline(0, color='black', linestyle='--')
    axs[1].set_ylabel("Y Error")
    axs[1].legend()

    axs[2].plot(error_pose_time, error_pose_yaw, label="Yaw Error")
    axs[2].axhline(0, color='black', linestyle='--')
    axs[2].set_ylabel("Yaw Error")
    axs[2].legend()

    for ax in axs:
        ax.set_xlabel("Time (s)")
    plt.tight_layout()

    # Plot Figure 2: Reference and True States
    fig2, axs2 = plt.subplots(3, 1, figsize=(10, 10))
    fig2.suptitle('Reference and MPC States')

    axs2[0].plot(ref_pose_time, ref_pose_x, 'r-', label="Reference X")
    axs2[0].plot(pose_gt_time, pose_gt_x, 'b-', label="MPC X")
    axs2[0].legend()
    axs2[0].set_ylabel("X Position")

    axs2[1].plot(ref_pose_time, ref_pose_y, 'r-', label="Reference Y")
    axs2[1].plot(pose_gt_time, pose_gt_y, 'b-', label="MPC Y")
    axs2[1].legend()
    axs2[1].set_ylabel("Y Position")

    axs2[2].plot(ref_pose_time, ref_pose_yaw, 'r-', label="Reference Yaw")
    axs2[2].plot(pose_gt_time, pose_gt_yaw, 'b-', label="MPC Yaw")
    axs2[2].legend()
    axs2[2].set_ylabel("Yaw")

    for ax in axs2:
        ax.set_xlabel("Time (s)")
    plt.tight_layout()

    # Plot Figure 3: Control Inputs
    fig3, axs3 = plt.subplots(2, 1, figsize=(10, 6))
    fig3.suptitle('Control Inputs')

    axs3[0].plot(control_inputs_time, left_thrust_angle, 'r-', label="Left Thrust Angle")
    axs3[0].plot(control_inputs_time, right_thrust_angle, 'b-', label="Right Thrust Angle")
    axs3[0].legend()
    axs3[0].set_ylabel("Thrust Angle (rad)")

    axs3[1].plot(control_inputs_time, left_thrust_cmd, 'r-', label="Left Thrust Command")
    axs3[1].plot(control_inputs_time, right_thrust_cmd, 'b-', label="Right Thrust Command")
    axs3[1].legend()
    axs3[1].set_ylabel("Thrust Command")

    for ax in axs3:
        ax.set_xlabel("Time (s)")
    plt.tight_layout()

    # Plot Figure 4: Trajectory in XY Plane
    fig4, ax4 = plt.subplots(figsize=(10, 8))
    fig4.suptitle('Trajectory in XY Plane')

    ax4.plot(ref_pose_x, ref_pose_y, 'r-', label="Reference Trajectory")
    ax4.plot(pose_gt_x, pose_gt_y, 'b-', label="MPC Trajectory")
    ax4.set_xlabel("X Position")
    ax4.set_ylabel("Y Position")
    ax4.legend()

    plt.tight_layout()
    # plt.show()

    # Plot Figure 4: ekf states
    fig5, axs5 = plt.subplots(3, 1, figsize=(10, 10))
    fig5.suptitle('Ground truth and EKF States')

    axs5[0].plot(pose_gt_time, pose_gt_x, 'b-', label="Ground truth X")
    axs5[0].plot(ekf_pose_time, ekf_pose_x, 'r-', label="EKF X")
    axs5[0].legend()
    axs5[0].set_ylabel("X Position")

    axs5[1].plot(pose_gt_time, pose_gt_y, 'b-', label="Ground truth Y")
    axs5[1].plot(ekf_pose_time, ekf_pose_y, 'r-', label="EKF Y")
    axs5[1].legend()
    axs5[1].set_ylabel("Y Position")

    axs5[2].plot(pose_gt_time, pose_gt_yaw, 'b-', label="Ground truth Yaw")
    axs5[2].plot(ekf_pose_time, ekf_pose_yaw, 'r-', label="EKF Yaw")
    axs5[2].legend()
    axs5[2].set_ylabel("Yaw")

    for ax in axs5:
        ax.set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()

def main():
    # Specify your bag folder path
    bag_dir = 'forward0312_0'  # Adjust this to your actual path, e.g., '/path/to/forward0303_0'

    # Initialize rclpy for message deserialization
    rclpy.init()

    # Read data from bag
    data = read_bag_data(bag_dir)

    # Plot the data
    plot_data(*data)

    # Shutdown rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()