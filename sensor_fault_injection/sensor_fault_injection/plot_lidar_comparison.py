#!/usr/bin/env python3
"""
LiDAR Fault Analysis Script for VRX USV Simulation

This script reads LiDAR point cloud data from a ROS2 bag file and visualizes
a comparison between original and faulty point clouds.

Usage:
    python3 plot_lidar_comparison.py <path_to_rosbag_folder>
    python3 plot_lidar_comparison.py <path_to_rosbag_folder> --frame 50
    python3 plot_lidar_comparison.py <path_to_rosbag_folder> --output plot.png
    python3 plot_lidar_comparison.py <path_to_rosbag_folder> --view top

Author: USV Fault Injection Research
"""

import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple, Optional
import struct

# ROS2 bag reading
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class PointCloudFrame:
    """Point cloud data for a single frame"""
    timestamp: float          # seconds
    points: np.ndarray        # Nx3 array of (x, y, z)
    intensities: Optional[np.ndarray] = None  # N array of intensity values
    num_points: int = 0


# =============================================================================
# Point Cloud Parsing Functions
# =============================================================================

def parse_point_cloud2(msg, typestore) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    Parse PointCloud2 message to numpy arrays.
    Filters out NaN and Inf values.
    """
    # Get field information
    fields = {f.name: f for f in msg.fields}
    
    # Check for required fields
    if 'x' not in fields or 'y' not in fields or 'z' not in fields:
        return np.array([]).reshape(0, 3), None
    
    # Calculate point step and offsets
    point_step = msg.point_step
    data = bytes(msg.data)
    
    # Get field offsets and datatypes
    x_offset = fields['x'].offset
    y_offset = fields['y'].offset
    z_offset = fields['z'].offset
    
    # Intensity field (optional)
    has_intensity = 'intensity' in fields
    if has_intensity:
        i_offset = fields['intensity'].offset
        i_datatype = fields['intensity'].datatype
    
    # Parse points
    num_points = msg.width * msg.height
    points = []
    intensities = []
    
    for i in range(num_points):
        offset = i * point_step
        
        # Extract x, y, z (assuming float32)
        x = struct.unpack_from('f', data, offset + x_offset)[0]
        y = struct.unpack_from('f', data, offset + y_offset)[0]
        z = struct.unpack_from('f', data, offset + z_offset)[0]
        
        # Skip NaN and Inf points - THIS IS THE KEY FIX
        if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
            continue
        
        points.append([x, y, z])
        
        # Extract intensity if available
        if has_intensity:
            # Datatype 7 = float32, 2 = uint8, 4 = uint16, 6 = uint32
            if i_datatype == 7:  # FLOAT32
                intensity = struct.unpack_from('f', data, offset + i_offset)[0]
            elif i_datatype == 2:  # UINT8
                intensity = struct.unpack_from('B', data, offset + i_offset)[0]
            elif i_datatype == 4:  # UINT16
                intensity = struct.unpack_from('H', data, offset + i_offset)[0]
            elif i_datatype == 6:  # UINT32
                intensity = struct.unpack_from('I', data, offset + i_offset)[0]
            else:
                intensity = 0
            
            # Also filter out NaN/Inf intensities
            if not np.isfinite(intensity):
                intensity = 0
            
            intensities.append(intensity)
    
    points_array = np.array(points) if points else np.array([]).reshape(0, 3)
    intensities_array = np.array(intensities) if intensities else None
    
    return points_array, intensities_array


# =============================================================================
# ROS2 Bag Reading Functions
# =============================================================================

def read_rosbag(bag_path: str,
                lidar_topic: str = '/wamv/sensors/lidars/lidar_wamv_sensor/points',
                lidar_faulty_topic: str = '/wamv/sensors/lidars/lidar_wamv_sensor/points_faulty',
                target_frame: int = -1
                ) -> Tuple[List[PointCloudFrame], List[PointCloudFrame]]:
    """
    Read LiDAR point cloud data from a ROS2 bag file.
    """
    original_frames: List[PointCloudFrame] = []
    faulty_frames: List[PointCloudFrame] = []
    
    bag_path = Path(bag_path)
    
    print(f"Reading rosbag from: {bag_path}")
    
    # Create typestore for message deserialization
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    with Reader(bag_path) as reader:
        # Print available topics
        print("\nAvailable topics in bag:")
        for topic, info in reader.topics.items():
            print(f"  {topic}: {info.msgtype} ({info.msgcount} messages)")
        
        print("\nReading point cloud messages...")
        
        for connection, timestamp, rawdata in reader.messages():
            topic = connection.topic
            
            # Convert timestamp to seconds
            time_sec = timestamp / 1e9
            
            if topic == lidar_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                points, intensities = parse_point_cloud2(msg, typestore)
                
                original_frames.append(PointCloudFrame(
                    timestamp=time_sec,
                    points=points,
                    intensities=intensities,
                    num_points=len(points)
                ))
                
            elif topic == lidar_faulty_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                points, intensities = parse_point_cloud2(msg, typestore)
                
                faulty_frames.append(PointCloudFrame(
                    timestamp=time_sec,
                    points=points,
                    intensities=intensities,
                    num_points=len(points)
                ))
    
    print(f"\nData loaded:")
    print(f"  LiDAR Original: {len(original_frames)} frames")
    print(f"  LiDAR Faulty: {len(faulty_frames)} frames")
    
    return original_frames, faulty_frames


def get_frame(frames: List[PointCloudFrame], frame_idx: int) -> Optional[PointCloudFrame]:
    """Get a specific frame or middle frame if -1"""
    if not frames:
        return None
    
    if frame_idx < 0:
        # Return middle frame
        return frames[len(frames) // 2]
    elif frame_idx < len(frames):
        return frames[frame_idx]
    else:
        print(f"Warning: Frame {frame_idx} out of range, using last frame")
        return frames[-1]


# =============================================================================
# Plotting Functions
# =============================================================================

def plot_side_by_side_3d(original: PointCloudFrame,
                          faulty: PointCloudFrame,
                          title: str = "LiDAR Point Cloud Comparison",
                          output_file: Optional[str] = None,
                          elev: float = 30,
                          azim: float = 45,
                          point_size: float = 0.5,
                          max_points: int = 50000):
    """
    Plot side-by-side 3D comparison of original and faulty point clouds.
    """
    fig = plt.figure(figsize=(16, 7))
    
    # Subsample if too many points
    def subsample(points, intensities, max_pts):
        if len(points) > max_pts:
            idx = np.random.choice(len(points), max_pts, replace=False)
            points = points[idx]
            if intensities is not None:
                intensities = intensities[idx]
        return points, intensities
    
    orig_points, orig_int = subsample(original.points.copy(), 
                                       original.intensities.copy() if original.intensities is not None else None, 
                                       max_points)
    faulty_points, faulty_int = subsample(faulty.points.copy() if len(faulty.points) > 0 else np.array([]).reshape(0,3), 
                                           faulty.intensities.copy() if faulty.intensities is not None else None, 
                                           max_points)
    
    # Original point cloud
    ax1 = fig.add_subplot(121, projection='3d')
    if len(orig_points) > 0:
        if orig_int is not None and len(orig_int) > 0:
            scatter1 = ax1.scatter(orig_points[:, 0], orig_points[:, 1], orig_points[:, 2],
                                   c=orig_int, cmap='viridis', s=point_size, alpha=0.6)
            plt.colorbar(scatter1, ax=ax1, label='Intensity', shrink=0.5)
        else:
            ax1.scatter(orig_points[:, 0], orig_points[:, 1], orig_points[:, 2],
                        c='blue', s=point_size, alpha=0.6)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title(f'Original ({original.num_points} valid points)')
    ax1.view_init(elev=elev, azim=azim)
    
    # Faulty point cloud
    ax2 = fig.add_subplot(122, projection='3d')
    if len(faulty_points) > 0:
        if faulty_int is not None and len(faulty_int) > 0:
            scatter2 = ax2.scatter(faulty_points[:, 0], faulty_points[:, 1], faulty_points[:, 2],
                                   c=faulty_int, cmap='viridis', s=point_size, alpha=0.6)
            plt.colorbar(scatter2, ax=ax2, label='Intensity', shrink=0.5)
        else:
            ax2.scatter(faulty_points[:, 0], faulty_points[:, 1], faulty_points[:, 2],
                        c='red', s=point_size, alpha=0.6)
    
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    ax2.set_title(f'Faulty ({faulty.num_points} valid points)')
    ax2.view_init(elev=elev, azim=azim)
    
    # Calculate axis limits from valid points only
    if len(orig_points) > 0 and len(faulty_points) > 0:
        all_points = np.vstack([orig_points, faulty_points])
    elif len(orig_points) > 0:
        all_points = orig_points
    elif len(faulty_points) > 0:
        all_points = faulty_points
    else:
        all_points = np.array([[0, 0, 0]])  # Default
    
    max_range = np.max(np.abs(all_points)) * 1.1
    z_min = all_points[:, 2].min() - 1
    z_max = all_points[:, 2].max() + 1
    
    for ax in [ax1, ax2]:
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([z_min, z_max])
    
    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"3D plot saved to: {output_file}")
    
    plt.show()


def plot_top_down(original: PointCloudFrame,
                   faulty: PointCloudFrame,
                   title: str = "LiDAR Point Cloud - Top Down View",
                   output_file: Optional[str] = None,
                   point_size: float = 0.5,
                   max_points: int = 50000):
    """
    Plot top-down (bird's eye) view comparison.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # Subsample if too many points
    def subsample(points, intensities, max_pts):
        if len(points) > max_pts:
            idx = np.random.choice(len(points), max_pts, replace=False)
            points = points[idx]
            if intensities is not None:
                intensities = intensities[idx]
        return points, intensities
    
    orig_points, orig_int = subsample(original.points.copy(),
                                       original.intensities.copy() if original.intensities is not None else None,
                                       max_points)
    faulty_points, faulty_int = subsample(faulty.points.copy() if len(faulty.points) > 0 else np.array([]).reshape(0,3),
                                           faulty.intensities.copy() if faulty.intensities is not None else None,
                                           max_points)
    
    # Original - top down (X-Y plane)
    if len(orig_points) > 0:
        if orig_int is not None and len(orig_int) > 0:
            scatter1 = axes[0].scatter(orig_points[:, 0], orig_points[:, 1],
                                        c=orig_int, cmap='viridis', s=point_size, alpha=0.6)
            plt.colorbar(scatter1, ax=axes[0], label='Intensity')
        else:
            axes[0].scatter(orig_points[:, 0], orig_points[:, 1],
                            c='blue', s=point_size, alpha=0.6)
    
    axes[0].set_xlabel('X (m) - Forward')
    axes[0].set_ylabel('Y (m) - Left')
    axes[0].set_title(f'Original ({original.num_points} points)')
    axes[0].set_aspect('equal')
    axes[0].grid(True, alpha=0.3)
    
    # Faulty - top down
    if len(faulty_points) > 0:
        if faulty_int is not None and len(faulty_int) > 0:
            scatter2 = axes[1].scatter(faulty_points[:, 0], faulty_points[:, 1],
                                        c=faulty_int, cmap='viridis', s=point_size, alpha=0.6)
            plt.colorbar(scatter2, ax=axes[1], label='Intensity')
        else:
            axes[1].scatter(faulty_points[:, 0], faulty_points[:, 1],
                            c='red', s=point_size, alpha=0.6)
    
    axes[1].set_xlabel('X (m) - Forward')
    axes[1].set_ylabel('Y (m) - Left')
    axes[1].set_title(f'Faulty ({faulty.num_points} points)')
    axes[1].set_aspect('equal')
    axes[1].grid(True, alpha=0.3)
    
    # Calculate axis limits
    if len(orig_points) > 0 and len(faulty_points) > 0:
        all_points = np.vstack([orig_points, faulty_points])
    elif len(orig_points) > 0:
        all_points = orig_points
    elif len(faulty_points) > 0:
        all_points = faulty_points
    else:
        all_points = np.array([[0, 0, 0]])
    
    max_range = np.max(np.abs(all_points[:, :2])) * 1.1
    
    for ax in axes:
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
    
    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    
    if output_file:
        filepath = output_file.replace('.png', '_topdown.png')
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"Top-down plot saved to: {filepath}")
    
    plt.show()


def plot_overlay(original: PointCloudFrame,
                  faulty: PointCloudFrame,
                  title: str = "LiDAR Point Cloud - Overlay Comparison",
                  output_file: Optional[str] = None,
                  point_size: float = 0.5,
                  max_points: int = 30000):
    """
    Plot overlay of original and faulty point clouds (top-down view).
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Subsample if too many points
    def subsample(points, max_pts):
        if len(points) > max_pts:
            idx = np.random.choice(len(points), max_pts, replace=False)
            return points[idx]
        return points
    
    orig_points = subsample(original.points.copy(), max_points) if len(original.points) > 0 else np.array([]).reshape(0,3)
    faulty_points = subsample(faulty.points.copy(), max_points) if len(faulty.points) > 0 else np.array([]).reshape(0,3)
    
    # Plot original in blue
    if len(orig_points) > 0:
        ax.scatter(orig_points[:, 0], orig_points[:, 1],
                   c='blue', s=point_size, alpha=0.5, label=f'Original ({original.num_points} pts)')
    
    # Plot faulty in red
    if len(faulty_points) > 0:
        ax.scatter(faulty_points[:, 0], faulty_points[:, 1],
                   c='red', s=point_size, alpha=0.5, label=f'Faulty ({faulty.num_points} pts)')
    
    ax.set_xlabel('X (m) - Forward')
    ax.set_ylabel('Y (m) - Left')
    ax.set_title(title)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    
    # Calculate axis limits
    if len(orig_points) > 0 and len(faulty_points) > 0:
        all_points = np.vstack([orig_points, faulty_points])
    elif len(orig_points) > 0:
        all_points = orig_points
    elif len(faulty_points) > 0:
        all_points = faulty_points
    else:
        all_points = np.array([[0, 0, 0]])
    
    max_range = np.max(np.abs(all_points[:, :2])) * 1.1
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    
    plt.tight_layout()
    
    if output_file:
        filepath = output_file.replace('.png', '_overlay.png')
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"Overlay plot saved to: {filepath}")
    
    plt.show()


def plot_range_histogram(original: PointCloudFrame,
                          faulty: PointCloudFrame,
                          title: str = "Point Range Distribution",
                          output_file: Optional[str] = None):
    """
    Plot histogram of point ranges (distance from origin).
    """
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Calculate ranges
    orig_ranges = np.sqrt(np.sum(original.points**2, axis=1)) if len(original.points) > 0 else np.array([])
    faulty_ranges = np.sqrt(np.sum(faulty.points**2, axis=1)) if len(faulty.points) > 0 else np.array([])
    
    if len(orig_ranges) == 0 and len(faulty_ranges) == 0:
        print("No valid points to plot histogram")
        return
    
    # Determine bin range
    max_range_val = 0
    if len(orig_ranges) > 0:
        max_range_val = max(max_range_val, orig_ranges.max())
    if len(faulty_ranges) > 0:
        max_range_val = max(max_range_val, faulty_ranges.max())
    
    bins = np.linspace(0, max_range_val * 1.1, 50)
    
    # Plot histograms
    if len(orig_ranges) > 0:
        ax.hist(orig_ranges, bins=bins, alpha=0.5, label=f'Original (n={len(orig_ranges)})', color='blue')
    if len(faulty_ranges) > 0:
        ax.hist(faulty_ranges, bins=bins, alpha=0.5, label=f'Faulty (n={len(faulty_ranges)})', color='red')
    
    ax.set_xlabel('Range (m)')
    ax.set_ylabel('Point Count')
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Add statistics
    stats_text = ""
    if len(orig_ranges) > 0:
        stats_text += f"Original: mean={orig_ranges.mean():.1f}m, max={orig_ranges.max():.1f}m\n"
    if len(faulty_ranges) > 0:
        stats_text += f"Faulty: mean={faulty_ranges.mean():.1f}m, max={faulty_ranges.max():.1f}m\n"
        if len(orig_ranges) > 0:
            stats_text += f"Point reduction: {(1 - len(faulty_ranges)/len(orig_ranges))*100:.1f}%"
    
    ax.text(0.98, 0.98, stats_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    if output_file:
        filepath = output_file.replace('.png', '_range_hist.png')
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"Range histogram saved to: {filepath}")
    
    plt.show()


def print_statistics(original: PointCloudFrame, faulty: PointCloudFrame):
    """Print comparison statistics."""
    print("\n" + "="*60)
    print("Point Cloud Statistics (valid points only, no inf/nan)")
    print("="*60)
    
    print(f"\nOriginal Point Cloud:")
    print(f"  Valid points: {original.num_points}")
    if original.num_points > 0:
        ranges = np.sqrt(np.sum(original.points**2, axis=1))
        print(f"  Range: min={ranges.min():.2f}m, max={ranges.max():.2f}m, mean={ranges.mean():.2f}m")
        print(f"  X: min={original.points[:, 0].min():.2f}m, max={original.points[:, 0].max():.2f}m")
        print(f"  Y: min={original.points[:, 1].min():.2f}m, max={original.points[:, 1].max():.2f}m")
        print(f"  Z: min={original.points[:, 2].min():.2f}m, max={original.points[:, 2].max():.2f}m")
        if original.intensities is not None and len(original.intensities) > 0:
            print(f"  Intensity: min={original.intensities.min():.1f}, max={original.intensities.max():.1f}, mean={original.intensities.mean():.1f}")
    
    print(f"\nFaulty Point Cloud:")
    print(f"  Valid points: {faulty.num_points}")
    if faulty.num_points > 0:
        ranges = np.sqrt(np.sum(faulty.points**2, axis=1))
        print(f"  Range: min={ranges.min():.2f}m, max={ranges.max():.2f}m, mean={ranges.mean():.2f}m")
        print(f"  X: min={faulty.points[:, 0].min():.2f}m, max={faulty.points[:, 0].max():.2f}m")
        print(f"  Y: min={faulty.points[:, 1].min():.2f}m, max={faulty.points[:, 1].max():.2f}m")
        print(f"  Z: min={faulty.points[:, 2].min():.2f}m, max={faulty.points[:, 2].max():.2f}m")
        if faulty.intensities is not None and len(faulty.intensities) > 0:
            print(f"  Intensity: min={faulty.intensities.min():.1f}, max={faulty.intensities.max():.1f}, mean={faulty.intensities.mean():.1f}")
    
    if original.num_points > 0:
        point_diff = faulty.num_points - original.num_points
        point_pct = (faulty.num_points / original.num_points - 1) * 100
        print(f"\nComparison:")
        print(f"  Point count difference: {point_diff:+d} ({point_pct:+.1f}%)")


# =============================================================================
# Main Function
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Visualize LiDAR fault injection from ROS2 bag',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 plot_lidar_comparison.py /path/to/rosbag_folder
    python3 plot_lidar_comparison.py /path/to/rosbag_folder --frame 50
    python3 plot_lidar_comparison.py /path/to/rosbag_folder --output result.png
    python3 plot_lidar_comparison.py /path/to/rosbag_folder --view top
        """
    )
    
    parser.add_argument('bag_path', type=str, help='Path to ROS2 bag folder')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Output file for plots (e.g., result.png)')
    parser.add_argument('--frame', '-f', type=int, default=-1,
                        help='Frame index to visualize (-1 = middle frame)')
    parser.add_argument('--view', '-v', type=str, default='3d',
                        choices=['3d', 'top', 'overlay', 'histogram'],
                        help='View type: 3d, top (bird\'s eye), overlay, histogram')
    parser.add_argument('--all-views', '-a', action='store_true',
                        help='Generate all view types')
    parser.add_argument('--lidar-topic', type=str,
                        default='/wamv/sensors/lidars/lidar_wamv_sensor/points',
                        help='Original LiDAR topic name')
    parser.add_argument('--lidar-faulty-topic', type=str,
                        default='/wamv/sensors/lidars/lidar_wamv_sensor/points_faulty',
                        help='Faulty LiDAR topic name')
    parser.add_argument('--title', type=str, default='LiDAR Fault Injection Analysis',
                        help='Plot title')
    parser.add_argument('--max-points', type=int, default=50000,
                        help='Maximum points to display (subsampling for performance)')
    parser.add_argument('--point-size', type=float, default=0.5,
                        help='Point size for scatter plots')
    
    args = parser.parse_args()
    
    # Read rosbag
    original_frames, faulty_frames = read_rosbag(
        args.bag_path,
        lidar_topic=args.lidar_topic,
        lidar_faulty_topic=args.lidar_faulty_topic,
        target_frame=args.frame
    )
    
    if not original_frames:
        print("Error: No original LiDAR data found!")
        print(f"  Looked for topic: {args.lidar_topic}")
        sys.exit(1)
    
    # Get the specified frame
    original = get_frame(original_frames, args.frame)
    faulty = get_frame(faulty_frames, args.frame)
    
    if faulty is None:
        print("Warning: No faulty LiDAR data found, creating empty frame")
        faulty = PointCloudFrame(timestamp=0, points=np.array([]).reshape(0, 3), num_points=0)
    
    frame_idx = args.frame if args.frame >= 0 else len(original_frames) // 2
    print(f"\nVisualizing frame {frame_idx} (timestamp: {original.timestamp:.3f}s)")
    
    # Print statistics
    print_statistics(original, faulty)
    
    # Generate plots
    if args.all_views:
        print("\nGenerating all views...")
        plot_side_by_side_3d(original, faulty, title=f"{args.title} - 3D View",
                             output_file=args.output, max_points=args.max_points,
                             point_size=args.point_size)
        plot_top_down(original, faulty, title=f"{args.title} - Top Down",
                      output_file=args.output, max_points=args.max_points,
                      point_size=args.point_size)
        plot_overlay(original, faulty, title=f"{args.title} - Overlay",
                     output_file=args.output, max_points=args.max_points,
                     point_size=args.point_size)
        plot_range_histogram(original, faulty, title=f"{args.title} - Range Distribution",
                             output_file=args.output)
    else:
        if args.view == '3d':
            plot_side_by_side_3d(original, faulty, title=args.title,
                                 output_file=args.output, max_points=args.max_points,
                                 point_size=args.point_size)
        elif args.view == 'top':
            plot_top_down(original, faulty, title=args.title,
                          output_file=args.output, max_points=args.max_points,
                          point_size=args.point_size)
        elif args.view == 'overlay':
            plot_overlay(original, faulty, title=args.title,
                         output_file=args.output, max_points=args.max_points,
                         point_size=args.point_size)
        elif args.view == 'histogram':
            plot_range_histogram(original, faulty, title=args.title,
                                 output_file=args.output)


if __name__ == '__main__':
    main()
