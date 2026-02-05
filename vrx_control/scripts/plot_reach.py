#!/usr/bin/env python3

import os
import sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation

def extract_ghost_usv_poses(bag_path, output_world_snippet=True):
    """
    Extract exactly 5 USV poses matching the ghost USVs in plot_panelb.py
    
    Args:
        bag_path: Path to ROS2 bag directory
        output_world_snippet: If True, output Gazebo world XML snippet
    """
    
    # Check if bag exists
    if not os.path.exists(bag_path):
        print(f"Error: Bag path does not exist: {bag_path}")
        return None
    
    # Setup bag reader
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Get topic types
    topics_and_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topics_and_types}
    
    # Store odometry data
    odom_data = []
    
    print("Reading bag file...")
    
    # Read all messages
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        # Process odometry messages
        if topic == '/wamv/sensors/position/ground_truth_odometry':
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            
            # Convert quaternion to yaw
            q = msg.pose.pose.orientation
            rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
            euler = rotation.as_euler('xyz')
            
            odom_data.append({
                'time': timestamp * 1e-9,  # Convert to seconds
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z,
                'yaw': euler[2],
                'qx': q.x,
                'qy': q.y,
                'qz': q.z,
                'qw': q.w
            })
    
    if not odom_data:
        print("Error: No odometry data found in bag")
        return None
    
    print(f"Found {len(odom_data)} odometry messages")
    
    # Normalize time to start from 0
    start_time = odom_data[0]['time']
    for entry in odom_data:
        entry['time'] -= start_time
    
    # Match the logic from plot_panelb.py
    fault_time = 15.0  # When fault occurs
    mission_end_time = odom_data[-1]['time']
    
    # Generate exactly 5 ghost times as in plot_panelb.py
    num_ghosts = 5
    ghost_times = np.linspace(fault_time, mission_end_time, num_ghosts)
    
    # Find closest poses to ghost times
    extracted_poses = []
    for i, target_t in enumerate(ghost_times):
        # Find closest timestamp
        closest_idx = np.argmin([abs(entry['time'] - target_t) for entry in odom_data])
        pose = odom_data[closest_idx].copy()
        
        # Calculate time since fault for label
        time_since_fault = target_t - fault_time
        pose['time_since_fault'] = time_since_fault
        pose['target_time'] = target_t
        pose['actual_time'] = pose['time']
        pose['ghost_index'] = i + 1
        
        extracted_poses.append(pose)
    
    # Print results
    print("\n" + "="*60)
    print("EXTRACTED GHOST USV POSES (5 USVs)")
    print("="*60)
    
    for i, pose in enumerate(extracted_poses):
        progress = (pose['actual_time'] - fault_time) / (mission_end_time - fault_time)
        alpha = 0.3 + 0.5 * progress  # Match alpha calculation from plot_panelb
        
        print(f"\nGhost USV {i+1}:")
        print(f"  Time: {pose['actual_time']:.1f}s (t+{pose['time_since_fault']:.0f}s after fault)")
        print(f"  Position: x={pose['x']:.2f}, y={pose['y']:.2f}, z={pose['z']:.2f}")
        print(f"  Yaw: {np.degrees(pose['yaw']):.1f}°")
        print(f"  Alpha (transparency): {alpha:.2f}")
        print(f"  Progress: {progress*100:.1f}%")
    
    # Generate Gazebo world XML snippet for exactly 5 ghost USVs
    if output_world_snippet:
        print("\n" + "="*60)
        print("GAZEBO WORLD FILE SNIPPET (5 GHOST USVs)")
        print("="*60)
        print("<!-- Add these 5 ghost USVs to your sydney_regatta.sdf file -->\n")
        
        for i, pose in enumerate(extracted_poses):
            # Calculate transparency matching plot_panelb logic
            progress = (pose['actual_time'] - fault_time) / (mission_end_time - fault_time)
            alpha = 0.3 + 0.5 * progress
            transparency = 1.0 - alpha  # Gazebo uses opposite convention
            
            # Color based on progress (blue to green gradient)
            if i == 0:
                color = "0.2 0.2 0.8 1.0"  # Blue for earliest
            elif i == len(extracted_poses) - 1:
                color = "0.2 0.8 0.2 1.0"  # Green for latest
            else:
                # Gradient from blue to green
                r = 0.2
                g = 0.2 + (0.6 * progress)
                b = 0.8 - (0.6 * progress)
                color = f"{r:.1f} {g:.1f} {b:.1f} 1.0"
            
            print(f"""
<!-- Ghost USV {i+1} at t={pose['actual_time']:.1f}s (t+{pose['time_since_fault']:.0f}s after fault) -->
<model name="wamv_ghost_{i+1}">
  <static>true</static>
  <pose>{pose['x']:.3f} {pose['y']:.3f} {pose['z']:.3f} 0 0 {pose['yaw']:.3f}</pose>
  <include>
    <uri>model://wam-v</uri>
  </include>
  
  <!-- Visual properties for ghost effect -->
  <plugin name="ghost_visual_{i+1}" filename="libgazebo_visual_plugin.so">
    <update_rate>0</update_rate>
    <transparency>{transparency:.2f}</transparency>
    <ambient>{color}</ambient>
  </plugin>
</model>""")
    
    # Also generate a simpler version without plugins
    print("\n<!-- ALTERNATIVE: Simple static ghosts without plugins -->")
    for i, pose in enumerate(extracted_poses):
        print(f"""
<model name="wamv_ghost_{i+1}_simple">
  <static>true</static>
  <pose>{pose['x']:.3f} {pose['y']:.3f} {pose['z']:.3f} 0 0 {pose['yaw']:.3f}</pose>
  <include>
    <uri>model://wam-v</uri>
  </include>
</model>""")
    
    # Output spawn positions for use in launch file
    print("\n" + "="*60)
    print("SPAWN POSITIONS FOR LAUNCH FILE")
    print("="*60)
    print("# Add these positions to your competition.launch.py:")
    print("ghost_usv_positions = [")
    for i, pose in enumerate(extracted_poses):
        print(f"    [{pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f}, 0, 0, {pose['yaw']:.3f}],  "
              f"# Ghost {i+1} at t+{pose['time_since_fault']:.0f}s")
    print("]")
    
    # Output as Python data structure matching plot_panelb format
    print("\n" + "="*60)
    print("PYTHON DATA STRUCTURE (5 GHOST POSES)")
    print("="*60)
    print("# Matching the 5 ghost USVs from plot_panelb.py")
    print("ghost_poses = [")
    for i, pose in enumerate(extracted_poses):
        print(f"    {{'index': {i+1}, 'time': {pose['actual_time']:.2f}, "
              f"'time_since_fault': {pose['time_since_fault']:.1f}, "
              f"'x': {pose['x']:.2f}, 'y': {pose['y']:.2f}, "
              f"'yaw_deg': {np.degrees(pose['yaw']):.1f}}},")
    print("]")
    
    return extracted_poses

def save_poses_to_yaml(poses, filename="ghost_usv_config.yaml"):
    """Save the 5 ghost poses to a YAML configuration file"""
    with open(filename, 'w') as f:
        f.write("# Ghost USV Configuration - 5 poses from trajectory\n")
        f.write("# Generated from bag file analysis\n\n")
        f.write("models:\n")
        
        for i, pose in enumerate(poses):
            f.write(f"  - name: \"wamv_ghost_{i+1}\"\n")
            f.write(f"    type: \"wam-v\"\n")
            f.write(f"    static: true\n")
            f.write(f"    pose: [{pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f}, "
                   f"0, 0, {pose['yaw']:.3f}]\n")
            f.write(f"    # Time: t={pose['actual_time']:.1f}s "
                   f"(t+{pose['time_since_fault']:.0f}s after fault)\n\n")
    
    print(f"\nYAML configuration saved to {filename}")

def main():
    # Specify your bag path here
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    else:
        # Default path - update this to your bag location
        bag_path = "/home/yang/usv_ws/experiments/bags/env_mpc_left_0.95_20250907_043328"
        print(f"Using default bag path: {bag_path}")
        print("You can specify a different bag: python extract_ghost_poses.py /path/to/bag\n")
    
    # Extract exactly 5 ghost poses
    poses = extract_ghost_usv_poses(bag_path, output_world_snippet=True)
    
    if poses:
        # Save to YAML configuration file
        save_poses_to_yaml(poses)
        
        # Save to simple text file
        output_file = "ghost_5_usv_poses.txt"
        with open(output_file, 'w') as f:
            f.write("# 5 Ghost USV Poses matching plot_panelb.py\n")
            f.write("# index, time(s), time_since_fault(s), x(m), y(m), yaw(rad)\n")
            for i, pose in enumerate(poses):
                f.write(f"{i+1}, {pose['actual_time']:.2f}, {pose['time_since_fault']:.1f}, "
                       f"{pose['x']:.3f}, {pose['y']:.3f}, {pose['yaw']:.3f}\n")
        print(f"Poses saved to {output_file}")
        
        # Display summary
        print("\n" + "="*60)
        print("SUMMARY: 5 GHOST USVs EXTRACTED")
        print("="*60)
        print(f"Fault time: {15.0:.1f}s")
        print(f"Mission end: {poses[-1]['actual_time']:.1f}s")
        print(f"Recovery duration: {poses[-1]['time_since_fault']:.1f}s")
        print(f"Ghost USVs span from t={poses[0]['actual_time']:.1f}s to t={poses[-1]['actual_time']:.1f}s")

if __name__ == "__main__":
    main()