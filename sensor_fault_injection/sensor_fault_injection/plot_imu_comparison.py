#!/usr/bin/env python3
"""
IMU Fault Analysis Script for VRX USV Simulation

This script reads IMU and ground truth data from a ROS2 bag file and plots:
  1. Raw IMU comparison (original vs faulty):
     - Angular velocity (x, y, z) over time
     - Linear acceleration (x, y, z) over time
  2. Orientation comparison:
     - Roll, pitch, yaw vs ground truth

Usage:
    python3 plot_imu_comparison.py <path_to_rosbag_folder>
    python3 plot_imu_comparison.py <path_to_rosbag_folder> --output plot.png
    python3 plot_imu_comparison.py <path_to_rosbag_folder> --orientation-only
    python3 plot_imu_comparison.py <path_to_rosbag_folder> --raw-only

Author: USV Fault Injection Research
"""

import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple, Optional

# ROS2 bag reading
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class IMUPoint:
    """IMU data point"""
    timestamp: float          # seconds
    # Angular velocity (rad/s)
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    # Linear acceleration (m/s²)
    linear_acceleration_x: float
    linear_acceleration_y: float
    linear_acceleration_z: float
    # Orientation quaternion
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float


@dataclass
class OdomPoint:
    """Odometry/Ground truth data point with orientation"""
    timestamp: float  # seconds
    x: float          # meters
    y: float          # meters
    z: float          # meters
    # Orientation quaternion
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float


@dataclass
class EulerAngles:
    """Euler angles in radians"""
    timestamp: float
    roll: float   # rotation around x-axis
    pitch: float  # rotation around y-axis
    yaw: float    # rotation around z-axis


# =============================================================================
# Quaternion to Euler Conversion
# =============================================================================

def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
        x, y, z, w: Quaternion components
        
    Returns:
        (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def imu_to_euler(imu_points: List[IMUPoint]) -> List[EulerAngles]:
    """Convert IMU orientation quaternions to Euler angles."""
    euler_list = []
    for imu in imu_points:
        roll, pitch, yaw = quaternion_to_euler(
            imu.orientation_x, imu.orientation_y,
            imu.orientation_z, imu.orientation_w
        )
        euler_list.append(EulerAngles(
            timestamp=imu.timestamp,
            roll=roll,
            pitch=pitch,
            yaw=yaw
        ))
    return euler_list


def odom_to_euler(odom_points: List[OdomPoint]) -> List[EulerAngles]:
    """Convert odometry orientation quaternions to Euler angles."""
    euler_list = []
    for odom in odom_points:
        roll, pitch, yaw = quaternion_to_euler(
            odom.orientation_x, odom.orientation_y,
            odom.orientation_z, odom.orientation_w
        )
        euler_list.append(EulerAngles(
            timestamp=odom.timestamp,
            roll=roll,
            pitch=pitch,
            yaw=yaw
        ))
    return euler_list


# =============================================================================
# ROS2 Bag Reading Functions
# =============================================================================

def read_rosbag(bag_path: str,
                imu_topic: str = '/wamv/sensors/imu/imu/data',
                imu_faulty_topic: str = '/wamv/sensors/imu/imu/data_faulty',
                odom_topic: str = '/wamv/sensors/position/ground_truth_odometry'
                ) -> Tuple[List[IMUPoint], List[IMUPoint], List[OdomPoint]]:
    """
    Read IMU and odometry data from a ROS2 bag file.
    
    Args:
        bag_path: Path to the rosbag folder
        imu_topic: Topic name for original IMU data
        imu_faulty_topic: Topic name for faulty IMU data
        odom_topic: Topic name for ground truth odometry
        
    Returns:
        Tuple of (imu_original, imu_faulty, ground_truth)
    """
    imu_original: List[IMUPoint] = []
    imu_faulty: List[IMUPoint] = []
    ground_truth: List[OdomPoint] = []
    
    bag_path = Path(bag_path)
    
    print(f"Reading rosbag from: {bag_path}")
    
    # Create typestore for message deserialization
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    with Reader(bag_path) as reader:
        # Print available topics
        print("\nAvailable topics in bag:")
        for topic, info in reader.topics.items():
            print(f"  {topic}: {info.msgtype} ({info.msgcount} messages)")
        
        print("\nReading messages...")
        
        for connection, timestamp, rawdata in reader.messages():
            topic = connection.topic
            
            # Convert timestamp to seconds
            time_sec = timestamp / 1e9
            
            if topic == imu_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                imu_original.append(IMUPoint(
                    timestamp=time_sec,
                    angular_velocity_x=msg.angular_velocity.x,
                    angular_velocity_y=msg.angular_velocity.y,
                    angular_velocity_z=msg.angular_velocity.z,
                    linear_acceleration_x=msg.linear_acceleration.x,
                    linear_acceleration_y=msg.linear_acceleration.y,
                    linear_acceleration_z=msg.linear_acceleration.z,
                    orientation_x=msg.orientation.x,
                    orientation_y=msg.orientation.y,
                    orientation_z=msg.orientation.z,
                    orientation_w=msg.orientation.w
                ))
                
            elif topic == imu_faulty_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                imu_faulty.append(IMUPoint(
                    timestamp=time_sec,
                    angular_velocity_x=msg.angular_velocity.x,
                    angular_velocity_y=msg.angular_velocity.y,
                    angular_velocity_z=msg.angular_velocity.z,
                    linear_acceleration_x=msg.linear_acceleration.x,
                    linear_acceleration_y=msg.linear_acceleration.y,
                    linear_acceleration_z=msg.linear_acceleration.z,
                    orientation_x=msg.orientation.x,
                    orientation_y=msg.orientation.y,
                    orientation_z=msg.orientation.z,
                    orientation_w=msg.orientation.w
                ))
                
            elif topic == odom_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                ground_truth.append(OdomPoint(
                    timestamp=time_sec,
                    x=msg.pose.pose.position.x,
                    y=msg.pose.pose.position.y,
                    z=msg.pose.pose.position.z,
                    orientation_x=msg.pose.pose.orientation.x,
                    orientation_y=msg.pose.pose.orientation.y,
                    orientation_z=msg.pose.pose.orientation.z,
                    orientation_w=msg.pose.pose.orientation.w
                ))
    
    print(f"\nData loaded:")
    print(f"  IMU Original: {len(imu_original)} points")
    print(f"  IMU Faulty: {len(imu_faulty)} points")
    print(f"  Ground Truth: {len(ground_truth)} points")
    
    return imu_original, imu_faulty, ground_truth


# =============================================================================
# Plotting Functions
# =============================================================================

def plot_angular_velocity(imu_original: List[IMUPoint],
                          imu_faulty: List[IMUPoint],
                          title: str = "Angular Velocity Comparison",
                          output_file: Optional[str] = None):
    """
    Plot angular velocity (x, y, z) over time.
    """
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    # Normalize timestamps
    t0 = min(
        imu_original[0].timestamp if imu_original else float('inf'),
        imu_faulty[0].timestamp if imu_faulty else float('inf')
    )
    
    # Original IMU
    orig_t = [p.timestamp - t0 for p in imu_original]
    orig_wx = [p.angular_velocity_x for p in imu_original]
    orig_wy = [p.angular_velocity_y for p in imu_original]
    orig_wz = [p.angular_velocity_z for p in imu_original]
    
    # Faulty IMU
    faulty_t = [p.timestamp - t0 for p in imu_faulty]
    faulty_wx = [p.angular_velocity_x for p in imu_faulty]
    faulty_wy = [p.angular_velocity_y for p in imu_faulty]
    faulty_wz = [p.angular_velocity_z for p in imu_faulty]
    
    # Plot X (Roll rate)
    axes[0].plot(orig_t, orig_wx, 'g-', linewidth=1, label='Original', alpha=0.8)
    axes[0].plot(faulty_t, faulty_wx, 'r-', linewidth=1, label='Faulty', alpha=0.7)
    axes[0].set_ylabel('ωx (rad/s)', fontsize=11)
    axes[0].set_title('Angular Velocity X (Roll Rate)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    
    # Plot Y (Pitch rate)
    axes[1].plot(orig_t, orig_wy, 'g-', linewidth=1, alpha=0.8)
    axes[1].plot(faulty_t, faulty_wy, 'r-', linewidth=1, alpha=0.7)
    axes[1].set_ylabel('ωy (rad/s)', fontsize=11)
    axes[1].set_title('Angular Velocity Y (Pitch Rate)')
    axes[1].grid(True, alpha=0.3)
    
    # Plot Z (Yaw rate)
    axes[2].plot(orig_t, orig_wz, 'g-', linewidth=1, alpha=0.8)
    axes[2].plot(faulty_t, faulty_wz, 'r-', linewidth=1, alpha=0.7)
    axes[2].set_ylabel('ωz (rad/s)', fontsize=11)
    axes[2].set_title('Angular Velocity Z (Yaw Rate)')
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].grid(True, alpha=0.3)
    
    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    
    if output_file:
        filepath = output_file.replace('.png', '_angular_velocity.png')
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"Angular velocity plot saved to: {filepath}")
    
    plt.show()


def plot_linear_acceleration(imu_original: List[IMUPoint],
                             imu_faulty: List[IMUPoint],
                             title: str = "Linear Acceleration Comparison",
                             output_file: Optional[str] = None):
    """
    Plot linear acceleration (x, y, z) over time.
    """
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    # Normalize timestamps
    t0 = min(
        imu_original[0].timestamp if imu_original else float('inf'),
        imu_faulty[0].timestamp if imu_faulty else float('inf')
    )
    
    # Original IMU
    orig_t = [p.timestamp - t0 for p in imu_original]
    orig_ax = [p.linear_acceleration_x for p in imu_original]
    orig_ay = [p.linear_acceleration_y for p in imu_original]
    orig_az = [p.linear_acceleration_z for p in imu_original]
    
    # Faulty IMU
    faulty_t = [p.timestamp - t0 for p in imu_faulty]
    faulty_ax = [p.linear_acceleration_x for p in imu_faulty]
    faulty_ay = [p.linear_acceleration_y for p in imu_faulty]
    faulty_az = [p.linear_acceleration_z for p in imu_faulty]
    
    # Plot X
    axes[0].plot(orig_t, orig_ax, 'g-', linewidth=1, label='Original', alpha=0.8)
    axes[0].plot(faulty_t, faulty_ax, 'r-', linewidth=1, label='Faulty', alpha=0.7)
    axes[0].set_ylabel('ax (m/s²)', fontsize=11)
    axes[0].set_title('Linear Acceleration X (Forward)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    
    # Plot Y
    axes[1].plot(orig_t, orig_ay, 'g-', linewidth=1, alpha=0.8)
    axes[1].plot(faulty_t, faulty_ay, 'r-', linewidth=1, alpha=0.7)
    axes[1].set_ylabel('ay (m/s²)', fontsize=11)
    axes[1].set_title('Linear Acceleration Y (Lateral)')
    axes[1].grid(True, alpha=0.3)
    
    # Plot Z
    axes[2].plot(orig_t, orig_az, 'g-', linewidth=1, alpha=0.8)
    axes[2].plot(faulty_t, faulty_az, 'r-', linewidth=1, alpha=0.7)
    axes[2].set_ylabel('az (m/s²)', fontsize=11)
    axes[2].set_title('Linear Acceleration Z (Vertical)')
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].grid(True, alpha=0.3)
    
    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    
    if output_file:
        filepath = output_file.replace('.png', '_linear_acceleration.png')
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"Linear acceleration plot saved to: {filepath}")
    
    plt.show()


def plot_orientation(imu_original: List[IMUPoint],
                     imu_faulty: List[IMUPoint],
                     ground_truth: List[OdomPoint],
                     title: str = "Orientation Comparison (Roll, Pitch, Yaw)",
                     output_file: Optional[str] = None):
    """
    Plot roll, pitch, yaw comparison between IMU original, faulty, and ground truth.
    """
    # Convert to Euler angles
    euler_original = imu_to_euler(imu_original)
    euler_faulty = imu_to_euler(imu_faulty)
    euler_gt = odom_to_euler(ground_truth)
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    # Normalize timestamps
    t0 = min(
        euler_original[0].timestamp if euler_original else float('inf'),
        euler_faulty[0].timestamp if euler_faulty else float('inf'),
        euler_gt[0].timestamp if euler_gt else float('inf')
    )
    
    # Ground truth
    gt_t = [p.timestamp - t0 for p in euler_gt]
    gt_roll = [np.degrees(p.roll) for p in euler_gt]
    gt_pitch = [np.degrees(p.pitch) for p in euler_gt]
    gt_yaw = [np.degrees(p.yaw) for p in euler_gt]
    
    # Original IMU
    orig_t = [p.timestamp - t0 for p in euler_original]
    orig_roll = [np.degrees(p.roll) for p in euler_original]
    orig_pitch = [np.degrees(p.pitch) for p in euler_original]
    orig_yaw = [np.degrees(p.yaw) for p in euler_original]
    
    # Faulty IMU
    faulty_t = [p.timestamp - t0 for p in euler_faulty]
    faulty_roll = [np.degrees(p.roll) for p in euler_faulty]
    faulty_pitch = [np.degrees(p.pitch) for p in euler_faulty]
    faulty_yaw = [np.degrees(p.yaw) for p in euler_faulty]
    
    # Plot Roll
    axes[0].plot(gt_t, gt_roll, 'b-', linewidth=1.5, label='Ground Truth', alpha=0.8)
    axes[0].plot(orig_t, orig_roll, 'g--', linewidth=1, label='IMU Original', alpha=0.7)
    axes[0].plot(faulty_t, faulty_roll, 'r:', linewidth=1, label='IMU Faulty', alpha=0.7)
    axes[0].set_ylabel('Roll (°)', fontsize=11)
    axes[0].set_title('Roll (Rotation around X-axis)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    
    # Plot Pitch
    axes[1].plot(gt_t, gt_pitch, 'b-', linewidth=1.5, alpha=0.8)
    axes[1].plot(orig_t, orig_pitch, 'g--', linewidth=1, alpha=0.7)
    axes[1].plot(faulty_t, faulty_pitch, 'r:', linewidth=1, alpha=0.7)
    axes[1].set_ylabel('Pitch (°)', fontsize=11)
    axes[1].set_title('Pitch (Rotation around Y-axis)')
    axes[1].grid(True, alpha=0.3)
    
    # Plot Yaw
    axes[2].plot(gt_t, gt_yaw, 'b-', linewidth=1.5, alpha=0.8)
    axes[2].plot(orig_t, orig_yaw, 'g--', linewidth=1, alpha=0.7)
    axes[2].plot(faulty_t, faulty_yaw, 'r:', linewidth=1, alpha=0.7)
    axes[2].set_ylabel('Yaw (°)', fontsize=11)
    axes[2].set_title('Yaw (Rotation around Z-axis / Heading)')
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].grid(True, alpha=0.3)
    
    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    
    if output_file:
        filepath = output_file.replace('.png', '_orientation.png')
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"Orientation plot saved to: {filepath}")
    
    plt.show()


def plot_message_rate(imu_original: List[IMUPoint],
                      imu_faulty: List[IMUPoint],
                      window_size: float = 1.0,
                      title: str = "IMU Message Rate Comparison",
                      output_file: Optional[str] = None):
    """
    Plot message rate (Hz) over time - useful for detecting degraded rate fault.
    
    Args:
        window_size: Time window in seconds for rate calculation
    """
    def calculate_rate(points: List[IMUPoint], window: float) -> Tuple[List[float], List[float]]:
        """Calculate instantaneous message rate over sliding window."""
        if len(points) < 2:
            return [], []
        
        times = []
        rates = []
        
        t0 = points[0].timestamp
        
        for i, p in enumerate(points):
            # Count messages in window ending at this point
            window_start = p.timestamp - window
            count = sum(1 for q in points[:i+1] if q.timestamp >= window_start)
            rate = count / window
            
            times.append(p.timestamp - t0)
            rates.append(rate)
        
        return times, rates
    
    fig, ax = plt.subplots(figsize=(14, 6))
    
    # Calculate rates
    orig_t, orig_rate = calculate_rate(imu_original, window_size)
    faulty_t, faulty_rate = calculate_rate(imu_faulty, window_size)
    
    # Plot
    ax.plot(orig_t, orig_rate, 'g-', linewidth=1.5, label='Original', alpha=0.8)
    ax.plot(faulty_t, faulty_rate, 'r-', linewidth=1.5, label='Faulty', alpha=0.8)
    
    # Add expected rate line if we can estimate it
    if orig_rate:
        expected_rate = np.median(orig_rate)
        ax.axhline(y=expected_rate, color='b', linestyle='--', 
                   label=f'Expected ({expected_rate:.1f} Hz)', alpha=0.5)
    
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Message Rate (Hz)', fontsize=11)
    ax.set_title(title, fontsize=14)
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_file:
        filepath = output_file.replace('.png', '_message_rate.png')
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"Message rate plot saved to: {filepath}")
    
    plt.show()
    
    # Print statistics
    if orig_rate and faulty_rate:
        print("\n" + "="*60)
        print("Message Rate Statistics")
        print("="*60)
        print(f"\nOriginal IMU:")
        print(f"  Mean rate: {np.mean(orig_rate):.2f} Hz")
        print(f"  Min rate:  {np.min(orig_rate):.2f} Hz")
        print(f"  Max rate:  {np.max(orig_rate):.2f} Hz")
        print(f"\nFaulty IMU:")
        print(f"  Mean rate: {np.mean(faulty_rate):.2f} Hz")
        print(f"  Min rate:  {np.min(faulty_rate):.2f} Hz")
        print(f"  Max rate:  {np.max(faulty_rate):.2f} Hz")
        print(f"\nRate reduction: {(1 - np.mean(faulty_rate)/np.mean(orig_rate))*100:.1f}%")


def plot_imu_error(imu_original: List[IMUPoint],
                   imu_faulty: List[IMUPoint],
                   title: str = "IMU Error (Faulty - Original)",
                   output_file: Optional[str] = None):
    """
    Plot the difference between faulty and original IMU readings.
    Useful for visualizing stuck or saturation faults.
    """
    # Match timestamps (approximate)
    def find_closest(target_time: float, points: List[IMUPoint]) -> Optional[IMUPoint]:
        if not points:
            return None
        closest = min(points, key=lambda p: abs(p.timestamp - target_time))
        if abs(closest.timestamp - target_time) < 0.1:  # Within 100ms
            return closest
        return None
    
    errors_t = []
    errors_wx = []
    errors_wy = []
    errors_wz = []
    errors_ax = []
    errors_ay = []
    errors_az = []
    
    t0 = imu_original[0].timestamp if imu_original else 0
    
    for orig in imu_original:
        faulty = find_closest(orig.timestamp, imu_faulty)
        if faulty:
            errors_t.append(orig.timestamp - t0)
            errors_wx.append(faulty.angular_velocity_x - orig.angular_velocity_x)
            errors_wy.append(faulty.angular_velocity_y - orig.angular_velocity_y)
            errors_wz.append(faulty.angular_velocity_z - orig.angular_velocity_z)
            errors_ax.append(faulty.linear_acceleration_x - orig.linear_acceleration_x)
            errors_ay.append(faulty.linear_acceleration_y - orig.linear_acceleration_y)
            errors_az.append(faulty.linear_acceleration_z - orig.linear_acceleration_z)
    
    if not errors_t:
        print("Warning: Could not match timestamps between original and faulty IMU")
        return
    
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    
    # Angular velocity error
    axes[0].plot(errors_t, errors_wx, 'r-', linewidth=1, label='ωx error', alpha=0.8)
    axes[0].plot(errors_t, errors_wy, 'g-', linewidth=1, label='ωy error', alpha=0.8)
    axes[0].plot(errors_t, errors_wz, 'b-', linewidth=1, label='ωz error', alpha=0.8)
    axes[0].set_ylabel('Angular Velocity Error (rad/s)', fontsize=11)
    axes[0].set_title('Angular Velocity Error (Faulty - Original)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # Linear acceleration error
    axes[1].plot(errors_t, errors_ax, 'r-', linewidth=1, label='ax error', alpha=0.8)
    axes[1].plot(errors_t, errors_ay, 'g-', linewidth=1, label='ay error', alpha=0.8)
    axes[1].plot(errors_t, errors_az, 'b-', linewidth=1, label='az error', alpha=0.8)
    axes[1].set_ylabel('Linear Acceleration Error (m/s²)', fontsize=11)
    axes[1].set_title('Linear Acceleration Error (Faulty - Original)')
    axes[1].set_xlabel('Time (s)', fontsize=11)
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    
    if output_file:
        filepath = output_file.replace('.png', '_error.png')
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"IMU error plot saved to: {filepath}")
    
    plt.show()
    
    # Print statistics
    print("\n" + "="*60)
    print("IMU Error Statistics")
    print("="*60)
    print(f"\nAngular Velocity Error:")
    print(f"  ωx: Mean={np.mean(errors_wx):.4f}, Max={np.max(np.abs(errors_wx)):.4f} rad/s")
    print(f"  ωy: Mean={np.mean(errors_wy):.4f}, Max={np.max(np.abs(errors_wy)):.4f} rad/s")
    print(f"  ωz: Mean={np.mean(errors_wz):.4f}, Max={np.max(np.abs(errors_wz)):.4f} rad/s")
    print(f"\nLinear Acceleration Error:")
    print(f"  ax: Mean={np.mean(errors_ax):.4f}, Max={np.max(np.abs(errors_ax)):.4f} m/s²")
    print(f"  ay: Mean={np.mean(errors_ay):.4f}, Max={np.max(np.abs(errors_ay)):.4f} m/s²")
    print(f"  az: Mean={np.mean(errors_az):.4f}, Max={np.max(np.abs(errors_az)):.4f} m/s²")


# =============================================================================
# Main Function
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Analyze IMU fault injection data from ROS2 bag',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 plot_imu_comparison.py /path/to/rosbag_folder
    python3 plot_imu_comparison.py /path/to/rosbag_folder --output result.png
    python3 plot_imu_comparison.py /path/to/rosbag_folder --orientation-only
    python3 plot_imu_comparison.py /path/to/rosbag_folder --raw-only
    python3 plot_imu_comparison.py /path/to/rosbag_folder --rate-plot
    python3 plot_imu_comparison.py /path/to/rosbag_folder --error-plot
        """
    )
    
    parser.add_argument('bag_path', type=str, help='Path to ROS2 bag folder')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Output file base name (e.g., result.png)')
    parser.add_argument('--raw-only', action='store_true',
                        help='Only plot raw IMU data (angular velocity and acceleration)')
    parser.add_argument('--orientation-only', action='store_true',
                        help='Only plot orientation (roll, pitch, yaw)')
    parser.add_argument('--rate-plot', '-r', action='store_true',
                        help='Also plot message rate (useful for degraded rate fault)')
    parser.add_argument('--error-plot', '-e', action='store_true',
                        help='Also plot IMU error (faulty - original)')
    parser.add_argument('--imu-topic', type=str,
                        default='/wamv/sensors/imu/imu/data',
                        help='Original IMU topic name')
    parser.add_argument('--imu-faulty-topic', type=str,
                        default='/wamv/sensors/imu/imu/data_faulty',
                        help='Faulty IMU topic name')
    parser.add_argument('--odom-topic', type=str,
                        default='/wamv/sensors/position/ground_truth_odometry',
                        help='Ground truth odometry topic name')
    parser.add_argument('--title', type=str, default='IMU Fault Injection Analysis',
                        help='Plot title prefix')
    
    args = parser.parse_args()
    
    # Read rosbag
    imu_original, imu_faulty, ground_truth = read_rosbag(
        args.bag_path,
        imu_topic=args.imu_topic,
        imu_faulty_topic=args.imu_faulty_topic,
        odom_topic=args.odom_topic
    )
    
    if not imu_original:
        print("Error: No original IMU data found!")
        sys.exit(1)
    
    # Determine what to plot
    plot_raw = not args.orientation_only
    plot_orientation_flag = not args.raw_only
    
    # Plot raw IMU data
    if plot_raw:
        print("\nGenerating angular velocity plot...")
        plot_angular_velocity(
            imu_original,
            imu_faulty,
            title=f"{args.title} - Angular Velocity",
            output_file=args.output
        )
        
        print("\nGenerating linear acceleration plot...")
        plot_linear_acceleration(
            imu_original,
            imu_faulty,
            title=f"{args.title} - Linear Acceleration",
            output_file=args.output
        )
    
    # Plot orientation comparison
    if plot_orientation_flag and ground_truth:
        print("\nGenerating orientation plot...")
        plot_orientation(
            imu_original,
            imu_faulty,
            ground_truth,
            title=f"{args.title} - Orientation",
            output_file=args.output
        )
    elif plot_orientation_flag and not ground_truth:
        print("Warning: No ground truth data found, skipping orientation plot")
    
    # Optional: Message rate plot
    if args.rate_plot:
        print("\nGenerating message rate plot...")
        plot_message_rate(
            imu_original,
            imu_faulty,
            title=f"{args.title} - Message Rate",
            output_file=args.output
        )
    
    # Optional: Error plot
    if args.error_plot:
        print("\nGenerating error plot...")
        plot_imu_error(
            imu_original,
            imu_faulty,
            title=f"{args.title} - Error",
            output_file=args.output
        )


if __name__ == '__main__':
    main()
