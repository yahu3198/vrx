#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon, Circle, Rectangle, Wedge, FancyArrow
from matplotlib.collections import LineCollection
import matplotlib.patheffects as path_effects
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import seaborn as sns
from scipy.spatial.transform import Rotation

# Set publication quality defaults for single column
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial']
plt.rcParams['font.size'] = 8
plt.rcParams['axes.linewidth'] = 1.0
plt.rcParams['lines.linewidth'] = 1.5
plt.rcParams['axes.labelsize'] = 8
plt.rcParams['xtick.labelsize'] = 7
plt.rcParams['ytick.labelsize'] = 7
plt.rcParams['legend.fontsize'] = 6

class ICRAMotivationFigure:
    def __init__(self, baseline_bag_path, proposed_bag_path):
        self.baseline_bag_path = baseline_bag_path
        self.proposed_bag_path = proposed_bag_path
        self.data = {}
        self.harbor_zones = self._define_harbor_zones()
        self.dock_areas = self._define_dock_areas()
        
    def _define_harbor_zones(self):
        """Define harbor zones from your code"""
        zones = [
            np.array([[-580, 258], [-572, 241], [-600, 236], [-600, 248], [-580, 258]]),
            np.array([[-570, 223], [-568, 209], [-595, 208], [-595, 220], [-570, 223]]),
            np.array([[-568, 192], [-593, 191], [-593, 183], [-579, 184], [-568, 192]])
        ]
        return zones
    
    def _define_dock_areas(self):
        """Define dock obstacles"""
        docks = [
            np.array([[-572, 241], [-570, 223], [-595, 220], [-600, 236], [-572, 241]]),
            np.array([[-568, 209], [-568, 192], [-593, 191], [-595, 208], [-568, 209]])
        ]
        return docks
    
    def read_bag(self, bag_path):
        """Read messages from ROS2 bag"""
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        topics_and_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topics_and_types}
        
        data = {'timestamps': [], 'trajectory': {'x': [], 'y': [], 'psi': []}}
        
        while reader.has_next():
            (topic, msg_data, timestamp) = reader.read_next()
            
            if topic == '/wamv/sensors/position/ground_truth_odometry':
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(msg_data, msg_type)
                
                data['timestamps'].append(timestamp * 1e-9)
                data['trajectory']['x'].append(msg.pose.pose.position.x)
                data['trajectory']['y'].append(msg.pose.pose.position.y)
                
                # Convert quaternion to yaw
                q = msg.pose.pose.orientation
                rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
                euler = rotation.as_euler('xyz')
                data['trajectory']['psi'].append(euler[2])
        
        # Convert to relative time
        if data['timestamps']:
            t0 = data['timestamps'][0]
            data['timestamps'] = [t - t0 for t in data['timestamps']]
        
        return data
    
    def rotate_coords(self, x, y):
        """Rotate coordinates 90 degrees anticlockwise"""
        x_rot = -y
        y_rot = x
        return x_rot, y_rot
    
    def draw_ship_symbol(self, ax, x, y, heading, alpha=0.8, color='blue', size_scale=0.6):
        """Draw compact ship symbol"""
        # Smaller ship dimensions for column width
        length = 8 * size_scale
        width = 4 * size_scale
        
        # Create ship shape vertices
        ship_vertices = np.array([
            [-length/2, -width/2],
            [-length/2, width/2],
            [length/3, width/2],
            [length/2, 0],
            [length/3, -width/2],
            [-length/2, -width/2]
        ])
        
        # Rotate ship to match heading
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        rotation_matrix = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        rotated_vertices = ship_vertices @ rotation_matrix.T
        
        # Translate to position
        rotated_vertices[:, 0] += x
        rotated_vertices[:, 1] += y
        
        # Choose color based on success/failure
        if color == 'red':
            fc = 'salmon'
            ec = 'darkred'
        else:
            fc = 'dodgerblue'
            ec = 'steelblue'
        
        ship_patch = Polygon(rotated_vertices[:-1], fc=fc, ec=ec, 
                    alpha=alpha, linewidth=1.0)
        ax.add_patch(ship_patch)
    
    def generate_figure(self, save_path='icra_motivation_single.pdf'):
        """Generate single panel trajectory comparison figure"""
        
        # Read bag data
        print("Loading data...")
        self.data['baseline'] = self.read_bag(self.baseline_bag_path)
        self.data['proposed'] = self.read_bag(self.proposed_bag_path)
        
        # Create figure - optimized for single column width
        fig, ax = plt.subplots(figsize=(3.5, 3.0))
        ax.set_aspect('equal')
        
        # Tighter limits for column width (same as before)
        ax.set_xlim(-260, -180)
        ax.set_ylim(-605, -520)
        
        # Draw harbor zones with proper labels
        for i, zone in enumerate(self.harbor_zones):
            zone_rotated = np.array([self.rotate_coords(p[0], p[1]) for p in zone])
            zone_patch = Polygon(zone_rotated, fc='lightgreen', ec='darkgreen', 
                            alpha=0.25, linewidth=1.0, linestyle='--')
            ax.add_patch(zone_patch)
            
            # Zone labels with "Zone" text
            center = zone_rotated[:-1].mean(axis=0)
            ax.text(center[0], center[1], f'Zone {i+1}', 
                fontsize=6, weight='bold', ha='center', va='center',
                color='darkgreen', alpha=0.8)
        
        # Draw dock areas with labels
        for i, dock in enumerate(self.dock_areas):
            dock_rotated = np.array([self.rotate_coords(p[0], p[1]) for p in dock])
            dock_patch = Polygon(dock_rotated, fc='lightcoral', ec='darkred',
                            alpha=0.3, linewidth=1.0, hatch='///')
            ax.add_patch(dock_patch)
            
            # Dock labels
            center = dock_rotated[:-1].mean(axis=0)
            ax.text(center[0], center[1], f'Dock {i+1}',
                fontsize=6, ha='center', va='center', color='darkred', alpha=0.8)
        
        fault_time = 15.0
        
        # Process baseline trajectory (FAILURE)
        baseline_data = self.data.get('baseline', {})
        if baseline_data and 'trajectory' in baseline_data:
            traj = baseline_data['trajectory']
            times = baseline_data['timestamps']
            
            # Convert to numpy arrays
            x_vals = np.array(traj['x'])
            y_vals = np.array(traj['y'])
            psi_vals = np.array(traj['psi'])
            times = np.array(times)
            
            # Rotate coordinates
            x_rot, y_rot = self.rotate_coords(x_vals, y_vals)
            
            # Find fault time index
            fault_idx = np.argmin(np.abs(times - fault_time))
            
            # Plot post-fault trajectory with red gradient
            points = np.array([x_rot[fault_idx:], y_rot[fault_idx:]]).T.reshape(-1, 1, 2)
            if len(points) > 1:
                segments = np.concatenate([points[:-1], points[1:]], axis=1)
                colors = plt.cm.Reds(np.linspace(0.3, 0.8, len(segments)))
                lc = LineCollection(segments, colors=colors, linewidth=2.0, alpha=0.7)
                ax.add_collection(lc)
            
            # Add drift indication
            if len(x_rot) > fault_idx + 20:
                drift_points = np.array([x_rot[-20:], y_rot[-20:]]).T.reshape(-1, 1, 2)
                drift_segments = np.concatenate([drift_points[:-1], drift_points[1:]], axis=1)
                drift_lc = LineCollection(drift_segments, colors='darkred', 
                                        linewidth=1.5, alpha=0.4, linestyles='dashed')
                ax.add_collection(drift_lc)
            
            # Draw 5 USV ghosts for baseline
            mission_end_time = times[-1] if len(times) > 0 else 125.5
            ghost_times = np.linspace(fault_time, mission_end_time, 5)
            for t in ghost_times:
                idx = np.argmin(np.abs(times - t))
                if idx < len(x_rot):
                    progress = (t - fault_time) / (mission_end_time - fault_time)
                    alpha = 0.3 + 0.5 * progress
                    self.draw_ship_symbol(ax, x_rot[idx], y_rot[idx], 
                                        psi_vals[idx] + np.pi/2, alpha=alpha, color='red')
            
            # Failure marker - RED X
            if len(x_rot) > 0:
                final_x = x_rot[-1]
                final_y = y_rot[-1]
                # Draw red X
                cross_size = 3
                ax.plot([final_x-cross_size, final_x+cross_size], 
                        [final_y-cross_size, final_y+cross_size], 
                        'r-', linewidth=3, alpha=0.8)
                ax.plot([final_x-cross_size, final_x+cross_size], 
                        [final_y+cross_size, final_y-cross_size], 
                        'r-', linewidth=3, alpha=0.8)
                
                # Add to legend
                ax.plot([], [], 'r-', linewidth=2, label='Baseline MPC (Failed)')
        
        # Process proposed trajectory (SUCCESS)
        proposed_data = self.data.get('proposed', {})
        if proposed_data and 'trajectory' in proposed_data:
            traj = proposed_data['trajectory']
            times = proposed_data['timestamps']
            
            # Convert to numpy arrays
            x_vals = np.array(traj['x'])
            y_vals = np.array(traj['y'])
            psi_vals = np.array(traj['psi'])
            times = np.array(times)
            
            # Rotate coordinates
            x_rot, y_rot = self.rotate_coords(x_vals, y_vals)
            
            # Find fault time index
            fault_idx = np.argmin(np.abs(times - fault_time))
            
            # Plot post-fault trajectory with blue gradient
            points = np.array([x_rot[fault_idx:], y_rot[fault_idx:]]).T.reshape(-1, 1, 2)
            if len(points) > 1:
                segments = np.concatenate([points[:-1], points[1:]], axis=1)
                colors = plt.cm.Blues(np.linspace(0.3, 0.9, len(segments)))
                lc = LineCollection(segments, colors=colors, linewidth=2.0, alpha=0.8)
                ax.add_collection(lc)
            
            # Draw 5 USV ghosts for proposed
            mission_end_time = times[-1] if len(times) > 0 else 125.5
            ghost_times = np.linspace(fault_time, mission_end_time, 5)
            for t in ghost_times:
                idx = np.argmin(np.abs(times - t))
                if idx < len(x_rot):
                    progress = (t - fault_time) / (mission_end_time - fault_time)
                    alpha = 0.3 + 0.5 * progress
                    self.draw_ship_symbol(ax, x_rot[idx], y_rot[idx], 
                                        psi_vals[idx] + np.pi/2, alpha=alpha, color='blue')
            
            # Success marker - GREEN CHECKMARK
            if len(x_rot) > 0:
                final_x = x_rot[-1]
                final_y = y_rot[-1]
                # Checkmark lines
                ax.plot([final_x-2, final_x], [final_y-2, final_y-3], 'g-', linewidth=3)
                ax.plot([final_x, final_x+3], [final_y-3, final_y+1], 'g-', linewidth=3)
                
                # Add to legend with green line
                ax.plot([], [], 'dodgerblue', linewidth=2, label='EAMPC (Success)')
            
            # Fault indicator with yellow box
            if fault_idx < len(x_rot):
                fault_x = x_rot[fault_idx]
                fault_y = y_rot[fault_idx]
                ax.plot(fault_x, fault_y, 'r*', markersize=12,
                       markeredgewidth=1.5, markeredgecolor='darkred')
                
                # Yellow box with fault description
                ax.annotate('Left Thruster\n95% Loss', 
                    xy=(fault_x, fault_y), 
                    xytext=(fault_x-35, fault_y+13),
                    fontsize=7, weight='bold', color='darkred',
                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.3',
                                    color='darkred', linewidth=1.2),
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", 
                            edgecolor="darkred", alpha=0.9))
        
        # NO TITLE - removed completely
        
        # Axis labels
        ax.set_xlabel('North (m)', fontsize=8)
        ax.set_ylabel('East (m)', fontsize=8)
        
        # Compact legend
        ax.legend(loc='upper right', fontsize=6, framealpha=0.9, ncol=1)
        
        # Grid
        ax.grid(True, alpha=0.2, linestyle=':', linewidth=0.5)
        ax.tick_params(labelsize=7)
        
        # Scale bar
        ax.plot([-255, -240], [-601, -601], 'k-', linewidth=2)
        ax.text(-247.5, -599, '15 m', ha='center', fontsize=6)
        
        # Adjust layout
        plt.tight_layout()
        
        # Save figure
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Single panel motivation figure saved to {save_path}")
        
        return fig

if __name__ == "__main__":
    # Update with your actual bag paths
    baseline_bag = "/home/yang/usv_ws/experiments/bags/baseline_mpc_left_0.95_20250907_043919"
    proposed_bag = "/home/yang/usv_ws/experiments/bags/env_mpc_left_0.95_20250907_043328"
    
    generator = ICRAMotivationFigure(baseline_bag, proposed_bag)
    fig = generator.generate_figure('icra_motivation_single.pdf')
    
    plt.show()