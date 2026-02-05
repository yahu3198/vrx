#!/usr/bin/env python3
"""
GPS Fault Analysis Script for VRX USV Simulation

This script reads GPS and ground truth data from a ROS2 bag file,
transforms GPS coordinates to local ENU (East-North-Up) frame,
and plots a comparison between:
  - Ground truth position
  - Original GPS position (in local coordinates)
  - Faulty GPS position (in local coordinates)

The local coordinate frame uses the first GPS fix as the origin,
matching the frame used by ground_truth_odometry in VRX.

Usage:
    python3 plot_gps_comparison.py <path_to_rosbag_folder>
    python3 plot_gps_comparison.py <path_to_rosbag_folder> --output plot.png
    python3 plot_gps_comparison.py <path_to_rosbag_folder> --error-plot

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
class GPSPoint:
    """GPS data point"""
    timestamp: float  # seconds
    latitude: float   # degrees
    longitude: float  # degrees
    altitude: float   # meters
    

@dataclass
class OdomPoint:
    """Odometry/Ground truth data point"""
    timestamp: float  # seconds
    x: float          # meters
    y: float          # meters
    z: float          # meters


# =============================================================================
# Coordinate Transformation Functions
# =============================================================================

def geodetic_to_enu(lat: float, lon: float, alt: float,
                    lat_ref: float, lon_ref: float, alt_ref: float) -> Tuple[float, float, float]:
    """
    Convert geodetic coordinates (lat, lon, alt) to local ENU coordinates.
    
    This uses the same transformation as ROS2/Gazebo for ground truth odometry.
    
    Args:
        lat, lon, alt: Target point in geodetic coordinates (degrees, degrees, meters)
        lat_ref, lon_ref, alt_ref: Reference/origin point (degrees, degrees, meters)
    
    Returns:
        (east, north, up) in meters relative to reference point
    """
    # WGS84 ellipsoid parameters
    a = 6378137.0  # Semi-major axis (meters)
    f = 1 / 298.257223563  # Flattening
    e2 = f * (2 - f)  # First eccentricity squared
    
    # Convert to radians
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    lat_ref_rad = np.radians(lat_ref)
    lon_ref_rad = np.radians(lon_ref)
    
    # Differences
    dlat = lat_rad - lat_ref_rad
    dlon = lon_rad - lon_ref_rad
    dalt = alt - alt_ref
    
    # Radius of curvature in the prime vertical
    N = a / np.sqrt(1 - e2 * np.sin(lat_ref_rad)**2)
    
    # Radius of curvature in the meridian
    M = a * (1 - e2) / (1 - e2 * np.sin(lat_ref_rad)**2)**1.5
    
    # ENU coordinates
    east = dlon * (N + alt_ref) * np.cos(lat_ref_rad)
    north = dlat * (M + alt_ref)
    up = dalt
    
    return east, north, up


def gps_to_local(gps_points: List[GPSPoint], 
                 lat_ref: float, lon_ref: float, alt_ref: float) -> List[OdomPoint]:
    """
    Convert a list of GPS points to local ENU coordinates.
    
    Args:
        gps_points: List of GPS data points
        lat_ref, lon_ref, alt_ref: Reference point for local frame origin
        
    Returns:
        List of OdomPoint in local coordinates
    """
    local_points = []
    
    for gps in gps_points:
        # Skip invalid GPS readings (NaN from outage)
        if np.isnan(gps.latitude) or np.isnan(gps.longitude):
            continue
            
        east, north, up = geodetic_to_enu(
            gps.latitude, gps.longitude, gps.altitude,
            lat_ref, lon_ref, alt_ref
        )
        
        local_points.append(OdomPoint(
            timestamp=gps.timestamp,
            x=east,
            y=north,
            z=up
        ))
    
    return local_points


# =============================================================================
# ROS2 Bag Reading Functions
# =============================================================================

def read_rosbag(bag_path: str,
                gps_topic: str = '/wamv/sensors/gps/gps/fix',
                gps_faulty_topic: str = '/wamv/sensors/gps/gps/fix_faulty',
                odom_topic: str = '/wamv/sensors/position/ground_truth_odometry'
                ) -> Tuple[List[GPSPoint], List[GPSPoint], List[OdomPoint]]:
    """
    Read GPS and odometry data from a ROS2 bag file.
    
    Args:
        bag_path: Path to the rosbag folder
        gps_topic: Topic name for original GPS data
        gps_faulty_topic: Topic name for faulty GPS data
        odom_topic: Topic name for ground truth odometry
        
    Returns:
        Tuple of (gps_original, gps_faulty, ground_truth)
    """
    gps_original: List[GPSPoint] = []
    gps_faulty: List[GPSPoint] = []
    ground_truth: List[OdomPoint] = []
    
    bag_path = Path(bag_path)
    
    print(f"Reading rosbag from: {bag_path}")
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
            
            if topic == gps_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                gps_original.append(GPSPoint(
                    timestamp=time_sec,
                    latitude=msg.latitude,
                    longitude=msg.longitude,
                    altitude=msg.altitude
                ))
                
            elif topic == gps_faulty_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                gps_faulty.append(GPSPoint(
                    timestamp=time_sec,
                    latitude=msg.latitude,
                    longitude=msg.longitude,
                    altitude=msg.altitude
                ))
                
            elif topic == odom_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                ground_truth.append(OdomPoint(
                    timestamp=time_sec,
                    x=msg.pose.pose.position.x,
                    y=msg.pose.pose.position.y,
                    z=msg.pose.pose.position.z
                ))
    
    print(f"\nData loaded:")
    print(f"  GPS Original: {len(gps_original)} points")
    print(f"  GPS Faulty: {len(gps_faulty)} points")
    print(f"  Ground Truth: {len(ground_truth)} points")
    
    return gps_original, gps_faulty, ground_truth


# =============================================================================
# Plotting Functions
# =============================================================================

def plot_xy_comparison(ground_truth: List[OdomPoint],
                       gps_local: List[OdomPoint],
                       gps_faulty_local: List[OdomPoint],
                       title: str = "GPS vs Ground Truth Comparison",
                       output_file: Optional[str] = None):
    """
    Plot X-Y trajectory comparison.
    
    Args:
        ground_truth: Ground truth odometry points
        gps_local: Original GPS in local coordinates
        gps_faulty_local: Faulty GPS in local coordinates
        title: Plot title
        output_file: If provided, save plot to this file
    """
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Extract coordinates
    gt_x = [p.x for p in ground_truth]
    gt_y = [p.y for p in ground_truth]
    
    gps_x = [p.x for p in gps_local]
    gps_y = [p.y for p in gps_local]
    
    faulty_x = [p.x for p in gps_faulty_local]
    faulty_y = [p.y for p in gps_faulty_local]
    
    # Plot trajectories
    ax.plot(gt_x, gt_y, 'b-', linewidth=2, label='Ground Truth', alpha=0.8)
    ax.plot(gps_x, gps_y, 'g--', linewidth=1.5, label='GPS Original', alpha=0.7)
    ax.plot(faulty_x, faulty_y, 'r:', linewidth=1.5, label='GPS Faulty', alpha=0.7)
    
    # Mark start and end points
    if gt_x:
        ax.plot(gt_x[0], gt_y[0], 'ko', markersize=10, label='Start', zorder=5)
        ax.plot(gt_x[-1], gt_y[-1], 'k^', markersize=10, label='End', zorder=5)
    
    # Labels and formatting
    ax.set_xlabel('East (m)', fontsize=12)
    ax.set_ylabel('North (m)', fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {output_file}")
    
    plt.show()


def plot_time_series(ground_truth: List[OdomPoint],
                     gps_local: List[OdomPoint],
                     gps_faulty_local: List[OdomPoint],
                     title: str = "Position Over Time",
                     output_file: Optional[str] = None):
    """
    Plot X, Y, Z positions over time.
    """
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    # Normalize timestamps to start from 0
    t0 = min(
        ground_truth[0].timestamp if ground_truth else float('inf'),
        gps_local[0].timestamp if gps_local else float('inf'),
        gps_faulty_local[0].timestamp if gps_faulty_local else float('inf')
    )
    
    # Ground truth
    gt_t = [p.timestamp - t0 for p in ground_truth]
    gt_x = [p.x for p in ground_truth]
    gt_y = [p.y for p in ground_truth]
    gt_z = [p.z for p in ground_truth]
    
    # GPS original
    gps_t = [p.timestamp - t0 for p in gps_local]
    gps_x = [p.x for p in gps_local]
    gps_y = [p.y for p in gps_local]
    gps_z = [p.z for p in gps_local]
    
    # GPS faulty
    faulty_t = [p.timestamp - t0 for p in gps_faulty_local]
    faulty_x = [p.x for p in gps_faulty_local]
    faulty_y = [p.y for p in gps_faulty_local]
    faulty_z = [p.z for p in gps_faulty_local]
    
    # Plot X
    axes[0].plot(gt_t, gt_x, 'b-', linewidth=1.5, label='Ground Truth', alpha=0.8)
    axes[0].plot(gps_t, gps_x, 'g--', linewidth=1, label='GPS Original', alpha=0.7)
    axes[0].plot(faulty_t, faulty_x, 'r:', linewidth=1, label='GPS Faulty', alpha=0.7)
    axes[0].set_ylabel('East X (m)', fontsize=11)
    axes[0].legend(loc='best')
    axes[0].grid(True, alpha=0.3)
    
    # Plot Y
    axes[1].plot(gt_t, gt_y, 'b-', linewidth=1.5, alpha=0.8)
    axes[1].plot(gps_t, gps_y, 'g--', linewidth=1, alpha=0.7)
    axes[1].plot(faulty_t, faulty_y, 'r:', linewidth=1, alpha=0.7)
    axes[1].set_ylabel('North Y (m)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    
    # Plot Z
    axes[2].plot(gt_t, gt_z, 'b-', linewidth=1.5, alpha=0.8)
    axes[2].plot(gps_t, gps_z, 'g--', linewidth=1, alpha=0.7)
    axes[2].plot(faulty_t, faulty_z, 'r:', linewidth=1, alpha=0.7)
    axes[2].set_ylabel('Up Z (m)', fontsize=11)
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].grid(True, alpha=0.3)
    
    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Time series plot saved to: {output_file}")
    
    plt.show()


def plot_position_error(ground_truth: List[OdomPoint],
                        gps_local: List[OdomPoint],
                        gps_faulty_local: List[OdomPoint],
                        title: str = "Position Error Over Time",
                        output_file: Optional[str] = None):
    """
    Plot position error (distance from ground truth) over time.
    """
    # Interpolate ground truth to GPS timestamps for error calculation
    def interpolate_gt(gt_points: List[OdomPoint], target_time: float) -> Optional[OdomPoint]:
        """Linear interpolation of ground truth at target time"""
        if not gt_points:
            return None
        
        # Find surrounding points
        for i in range(len(gt_points) - 1):
            if gt_points[i].timestamp <= target_time <= gt_points[i + 1].timestamp:
                t1, t2 = gt_points[i].timestamp, gt_points[i + 1].timestamp
                if t2 == t1:
                    return gt_points[i]
                
                alpha = (target_time - t1) / (t2 - t1)
                return OdomPoint(
                    timestamp=target_time,
                    x=gt_points[i].x + alpha * (gt_points[i + 1].x - gt_points[i].x),
                    y=gt_points[i].y + alpha * (gt_points[i + 1].y - gt_points[i].y),
                    z=gt_points[i].z + alpha * (gt_points[i + 1].z - gt_points[i].z)
                )
        return None
    
    # Calculate errors for original GPS
    gps_errors_t = []
    gps_errors_2d = []
    gps_errors_3d = []
    
    for gps in gps_local:
        gt = interpolate_gt(ground_truth, gps.timestamp)
        if gt:
            error_2d = np.sqrt((gps.x - gt.x)**2 + (gps.y - gt.y)**2)
            error_3d = np.sqrt((gps.x - gt.x)**2 + (gps.y - gt.y)**2 + (gps.z - gt.z)**2)
            gps_errors_t.append(gps.timestamp)
            gps_errors_2d.append(error_2d)
            gps_errors_3d.append(error_3d)
    
    # Calculate errors for faulty GPS
    faulty_errors_t = []
    faulty_errors_2d = []
    faulty_errors_3d = []
    
    for gps in gps_faulty_local:
        gt = interpolate_gt(ground_truth, gps.timestamp)
        if gt:
            error_2d = np.sqrt((gps.x - gt.x)**2 + (gps.y - gt.y)**2)
            error_3d = np.sqrt((gps.x - gt.x)**2 + (gps.y - gt.y)**2 + (gps.z - gt.z)**2)
            faulty_errors_t.append(gps.timestamp)
            faulty_errors_2d.append(error_2d)
            faulty_errors_3d.append(error_3d)
    
    # Normalize time
    t0 = min(gps_errors_t[0] if gps_errors_t else float('inf'),
             faulty_errors_t[0] if faulty_errors_t else float('inf'))
    
    gps_errors_t = [t - t0 for t in gps_errors_t]
    faulty_errors_t = [t - t0 for t in faulty_errors_t]
    
    # Plot
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    
    # 2D Error (Horizontal)
    axes[0].plot(gps_errors_t, gps_errors_2d, 'g-', linewidth=1.5, 
                 label=f'GPS Original (RMSE: {np.sqrt(np.mean(np.array(gps_errors_2d)**2)):.2f}m)', alpha=0.8)
    axes[0].plot(faulty_errors_t, faulty_errors_2d, 'r-', linewidth=1.5,
                 label=f'GPS Faulty (RMSE: {np.sqrt(np.mean(np.array(faulty_errors_2d)**2)):.2f}m)', alpha=0.8)
    axes[0].set_ylabel('Horizontal Error (m)', fontsize=11)
    axes[0].legend(loc='best')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('2D Horizontal Position Error')
    
    # 3D Error
    axes[1].plot(gps_errors_t, gps_errors_3d, 'g-', linewidth=1.5,
                 label=f'GPS Original (RMSE: {np.sqrt(np.mean(np.array(gps_errors_3d)**2)):.2f}m)', alpha=0.8)
    axes[1].plot(faulty_errors_t, faulty_errors_3d, 'r-', linewidth=1.5,
                 label=f'GPS Faulty (RMSE: {np.sqrt(np.mean(np.array(faulty_errors_3d)**2)):.2f}m)', alpha=0.8)
    axes[1].set_ylabel('3D Error (m)', fontsize=11)
    axes[1].set_xlabel('Time (s)', fontsize=11)
    axes[1].legend(loc='best')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('3D Position Error')
    
    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Error plot saved to: {output_file}")
    
    plt.show()
    
    # Print statistics
    print("\n" + "="*60)
    print("Position Error Statistics")
    print("="*60)
    
    if gps_errors_2d:
        print("\nGPS Original:")
        print(f"  2D RMSE: {np.sqrt(np.mean(np.array(gps_errors_2d)**2)):.3f} m")
        print(f"  2D Max:  {max(gps_errors_2d):.3f} m")
        print(f"  2D Mean: {np.mean(gps_errors_2d):.3f} m")
        print(f"  3D RMSE: {np.sqrt(np.mean(np.array(gps_errors_3d)**2)):.3f} m")
    
    if faulty_errors_2d:
        print("\nGPS Faulty:")
        print(f"  2D RMSE: {np.sqrt(np.mean(np.array(faulty_errors_2d)**2)):.3f} m")
        print(f"  2D Max:  {max(faulty_errors_2d):.3f} m")
        print(f"  2D Mean: {np.mean(faulty_errors_2d):.3f} m")
        print(f"  3D RMSE: {np.sqrt(np.mean(np.array(faulty_errors_3d)**2)):.3f} m")


# =============================================================================
# Main Function
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Analyze GPS fault injection data from ROS2 bag',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 plot_gps_comparison.py /path/to/rosbag_folder
    python3 plot_gps_comparison.py /path/to/rosbag_folder --output trajectory.png
    python3 plot_gps_comparison.py /path/to/rosbag_folder --error-plot
    python3 plot_gps_comparison.py /path/to/rosbag_folder --time-series --error-plot
        """
    )
    
    parser.add_argument('bag_path', type=str, help='Path to ROS2 bag folder')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Output file for trajectory plot (e.g., plot.png)')
    parser.add_argument('--time-series', '-t', action='store_true',
                        help='Also plot time series of X, Y, Z')
    parser.add_argument('--error-plot', '-e', action='store_true',
                        help='Also plot position error over time')
    parser.add_argument('--gps-topic', type=str, 
                        default='/wamv/sensors/gps/gps/fix',
                        help='Original GPS topic name')
    parser.add_argument('--gps-faulty-topic', type=str,
                        default='/wamv/sensors/gps/gps/fix_faulty',
                        help='Faulty GPS topic name')
    parser.add_argument('--odom-topic', type=str,
                        default='/wamv/sensors/position/ground_truth_odometry',
                        help='Ground truth odometry topic name')
    parser.add_argument('--title', type=str, default='GPS Fault Injection Analysis',
                        help='Plot title')
    
    args = parser.parse_args()
    
    # Read rosbag
    gps_original, gps_faulty, ground_truth = read_rosbag(
        args.bag_path,
        gps_topic=args.gps_topic,
        gps_faulty_topic=args.gps_faulty_topic,
        odom_topic=args.odom_topic
    )
    
    if not gps_original:
        print("Error: No original GPS data found!")
        sys.exit(1)
    
    if not ground_truth:
        print("Error: No ground truth data found!")
        sys.exit(1)
    
    # Use first valid GPS point as reference for local coordinate transformation
    # Filter out NaN values
    valid_gps = [g for g in gps_original if not (np.isnan(g.latitude) or np.isnan(g.longitude))]
    
    if not valid_gps:
        print("Error: No valid GPS readings found!")
        sys.exit(1)
    
    lat_ref = valid_gps[0].latitude
    lon_ref = valid_gps[0].longitude
    alt_ref = valid_gps[0].altitude
    
    print(f"\nReference point (local frame origin):")
    print(f"  Latitude:  {lat_ref:.8f}°")
    print(f"  Longitude: {lon_ref:.8f}°")
    print(f"  Altitude:  {alt_ref:.2f} m")
    
    # Transform GPS to local coordinates
    print("\nTransforming GPS to local ENU coordinates...")
    gps_local = gps_to_local(gps_original, lat_ref, lon_ref, alt_ref)
    gps_faulty_local = gps_to_local(gps_faulty, lat_ref, lon_ref, alt_ref)
    
    print(f"  GPS Original (local): {len(gps_local)} valid points")
    print(f"  GPS Faulty (local): {len(gps_faulty_local)} valid points")
    
    # Adjust ground truth to same origin
    # VRX ground truth is already in local frame, but we need to align origins
    # Find the ground truth point closest to the first GPS timestamp
    first_gps_time = gps_local[0].timestamp if gps_local else 0
    
    # Find offset between GPS local and ground truth at the start
    gt_at_start = None
    for gt in ground_truth:
        if gt.timestamp >= first_gps_time:
            gt_at_start = gt
            break
    
    if gt_at_start and gps_local:
        # Calculate offset
        offset_x = gps_local[0].x - gt_at_start.x
        offset_y = gps_local[0].y - gt_at_start.y
        offset_z = gps_local[0].z - gt_at_start.z
        
        print(f"\nOrigin alignment offset:")
        print(f"  X: {offset_x:.3f} m, Y: {offset_y:.3f} m, Z: {offset_z:.3f} m")
        
        # Apply offset to ground truth to align with GPS local frame
        ground_truth_aligned = [
            OdomPoint(
                timestamp=gt.timestamp,
                x=gt.x + offset_x,
                y=gt.y + offset_y,
                z=gt.z + offset_z
            )
            for gt in ground_truth
        ]
    else:
        ground_truth_aligned = ground_truth
    
    # Plot XY trajectory
    print("\nGenerating trajectory plot...")
    plot_xy_comparison(
        ground_truth_aligned,
        gps_local,
        gps_faulty_local,
        title=args.title,
        output_file=args.output
    )
    
    # Optional: Time series plot
    if args.time_series:
        print("\nGenerating time series plot...")
        ts_output = args.output.replace('.png', '_timeseries.png') if args.output else None
        plot_time_series(
            ground_truth_aligned,
            gps_local,
            gps_faulty_local,
            title=f"{args.title} - Time Series",
            output_file=ts_output
        )
    
    # Optional: Error plot
    if args.error_plot:
        print("\nGenerating error plot...")
        err_output = args.output.replace('.png', '_error.png') if args.output else None
        plot_position_error(
            ground_truth_aligned,
            gps_local,
            gps_faulty_local,
            title=f"{args.title} - Position Error",
            output_file=err_output
        )


if __name__ == '__main__':
    main()