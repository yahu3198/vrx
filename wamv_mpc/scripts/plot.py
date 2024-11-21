import rosbag
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion

bag = rosbag.Bag('test211124_2.bag')
times = {}

# Initialize lists for error_pose
error_pose_x = []
error_pose_y = []
error_pose_yaw = []
error_pose_time = []

# Initialize lists for ref_pose and pose_gt
ref_pose_x = []
ref_pose_y = []
ref_pose_yaw = []
ref_pose_time = []
pose_gt_x = []
pose_gt_y = []
pose_gt_yaw = []
pose_gt_time = []

# Initialize lists for control inputs
left_thrust_angle = []
right_thrust_angle = []
left_thrust_cmd = []
right_thrust_cmd = []
control_inputs_time = []

for topic, msg, t in bag.read_messages():
    if topic == '/wamv/error_pose':
        error_pose_x.append(msg.pose.pose.position.x)
        error_pose_y.append(msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        error_pose_yaw.append(yaw)
        error_pose_time.append(msg.header.stamp.to_sec())

    elif topic == '/wamv/ref_pose':
        ref_pose_x.append(msg.pose.pose.position.x)
        ref_pose_y.append(msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        ref_pose_yaw.append(yaw)
        ref_pose_time.append(msg.header.stamp.to_sec())

    elif topic == '/wamv/pose_gt':
        pose_gt_x.append(msg.pose.pose.position.x)
        pose_gt_y.append(msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        pose_gt_yaw.append(yaw)
        pose_gt_time.append(msg.header.stamp.to_sec())

    # elif topic == '/wamv/thrusters/left_thrust_angle':
    #     left_thrust_angle.append(msg.data)

    # elif topic == '/wamv/thrusters/right_thrust_angle':
    #     right_thrust_angle.append(msg.data)

    # elif topic == '/wamv/thrusters/left_thrust_cmd':
    #     left_thrust_cmd.append(msg.data)

    # elif topic == '/wamv/thrusters/right_thrust_cmd':
    #     right_thrust_cmd.append(msg.data)

    elif topic == '/wamv/control_inputs':
        left_thrust_angle.append(msg.twist.linear.x)
        left_thrust_cmd.append(msg.twist.linear.y)
        right_thrust_angle.append(msg.twist.angular.x)
        right_thrust_cmd.append(msg.twist.angular.y)
        control_inputs_time.append(msg.header.stamp.to_sec())

bag.close()

# Plot Figure 1: Error States
fig, axs = plt.subplots(3, 1, figsize=(10, 10))
fig.suptitle('Error States')

axs[0].plot(error_pose_time, error_pose_x, label="X Error")
axs[0].axhline(0, color='black', linestyle='--')
axs[0].set_ylabel("X Error")
# axs[0].set_ylim(-1, 1)
# axs[0].set_xlim(5, 30)

axs[1].plot(error_pose_time, error_pose_y, label="Y Error")
axs[1].axhline(0, color='black', linestyle='--')
axs[1].set_ylabel("Y Error")
# axs[1].set_ylim(-1, 1)
# axs[1].set_xlim(5, 30)

axs[2].plot(error_pose_time, error_pose_yaw, label="Yaw Error")
axs[2].axhline(0, color='black', linestyle='--')
axs[2].set_ylabel("Yaw Error")
# axs[2].set_ylim(-1, 1)
# axs[2].set_xlim(5, 30)

for ax in axs:
    ax.set_xlabel("Time (s)")
plt.tight_layout()

# Plot Figure 2: Reference and True States
fig2, axs2 = plt.subplots(3, 1, figsize=(10, 10))
fig2.suptitle('Reference and True States')

axs2[0].plot(ref_pose_time, ref_pose_x, 'r-', label="Reference X")
axs2[0].plot(pose_gt_time, pose_gt_x, 'b-', label="Ground Truth X")
axs2[0].legend()
axs2[0].set_ylabel("X Position")
# axs2[0].set_xlim(5, 30)

axs2[1].plot(ref_pose_time, ref_pose_y, 'r-', label="Reference Y")
axs2[1].plot(pose_gt_time, pose_gt_y, 'b-', label="Ground Truth Y")
axs2[1].legend()
axs2[1].set_ylabel("Y Position")
# axs2[1].set_xlim(5, 30)
# axs2[1].set_ylim(-1, 1)


axs2[2].plot(ref_pose_time, ref_pose_yaw, 'r-', label="Reference Yaw")
axs2[2].plot(pose_gt_time, pose_gt_yaw, 'b-', label="Ground Truth Yaw")
axs2[2].legend()
axs2[2].set_ylabel("Yaw")
# axs2[2].set_ylim(-1, 1)
# axs2[2].set_xlim(5, 30)

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
# axs3[0].set_xlim(5, 30)

axs3[1].plot(control_inputs_time, left_thrust_cmd, 'r-', label="Left Thrust Command")
axs3[1].plot(control_inputs_time, right_thrust_cmd, 'b-', label="Right Thrust Command")
axs3[1].legend()
axs3[1].set_ylabel("Thrust Command")
# axs3[1].set_xlim(5, 30)

plt.tight_layout()
plt.show()
