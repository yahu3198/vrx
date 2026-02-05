#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon, FancyBboxPatch, Circle, Wedge, FancyArrow
from matplotlib.collections import LineCollection
import matplotlib.patheffects as path_effects
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import seaborn as sns
from scipy.spatial.transform import Rotation

# Set publication quality defaults
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial']
plt.rcParams['font.size'] = 10
plt.rcParams['axes.linewidth'] = 1.5
plt.rcParams['lines.linewidth'] = 2
sns.set_palette("muted")

class PanelAGenerator:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.data = {}
        self.harbor_zones = self._define_harbor_zones()
        self.dock_areas = self._define_dock_areas()
        
    def _define_harbor_zones(self):
        """Define harbor zones from your code"""
        zones = [
            # Zone 1
            np.array([[-580, 258], [-572, 241], [-600, 236], [-600, 248], [-580, 258]]),
            # Zone 2  
            np.array([[-570, 223], [-568, 209], [-595, 208], [-595, 220], [-570, 223]]),
            # Zone 3
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
    
    def read_bag(self):
        """Read messages from ROS2 bag"""
        storage_options = StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        topics_and_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topics_and_types}
        
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            
            if topic not in self.data:
                self.data[topic] = {'timestamps': [], 'messages': []}
            
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            
            self.data[topic]['timestamps'].append(timestamp * 1e-9)
            self.data[topic]['messages'].append(msg)
    
    def extract_trajectory(self):
        """Extract USV trajectory and heading"""
        odom_data = self.data.get('/wamv/sensors/position/ground_truth_odometry', {})
        
        trajectory = {'time': [], 'x': [], 'y': [], 'psi': []}
        
        for i, msg in enumerate(odom_data.get('messages', [])):
            trajectory['time'].append(odom_data['timestamps'][i] - odom_data['timestamps'][0])
            trajectory['x'].append(msg.pose.pose.position.x)
            trajectory['y'].append(msg.pose.pose.position.y)
            
            # Convert quaternion to yaw
            q = msg.pose.pose.orientation
            rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
            euler = rotation.as_euler('xyz')
            trajectory['psi'].append(euler[2])
        
        return pd.DataFrame(trajectory)
    
    def draw_ship_symbol(self, ax, x, y, heading, time_label, alpha=0.8, color='red'):
        """Draw simplified ship symbol (rectangle + triangle) in RED for failure"""
        # Ship dimensions
        length = 10
        width = 5
        
        # Create ship shape vertices
        ship_vertices = np.array([
            [-length/2, -width/2],  # Stern port
            [-length/2, width/2],   # Stern starboard
            [length/3, width/2],    # Mid starboard
            [length/2, 0],          # Bow (triangle point)
            [length/3, -width/2],   # Mid port
            [-length/2, -width/2]   # Close shape
        ])
        
        # Rotate ship to match heading
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        rotation_matrix = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        
        rotated_vertices = ship_vertices @ rotation_matrix.T
        
        # Translate to position
        rotated_vertices[:, 0] += x
        rotated_vertices[:, 1] += y
        
        # Draw ship in RED for failure case
        ship_patch = Polygon(rotated_vertices[:-1], fc='salmon', ec='darkred', 
                    alpha=alpha, linewidth=1.5)
        ax.add_patch(ship_patch)
        
        # Add time label
        if time_label is not None:
            text = ax.text(x + 0.5, y + 1.5, time_label,
                        fontsize=8, alpha=alpha*0.7, weight='bold')
            text.set_path_effects([path_effects.withStroke(linewidth=3, foreground='white')])
    
    def generate_panel_a(self, save_path='figure1_panel_a.pdf'):
        """Generate Panel A showing baseline MPC failure"""
        
        # Extract data
        trajectory = self.extract_trajectory()
        
        # Create figure
        fig, ax = plt.subplots(figsize=(10, 6), dpi=150)  # Was (8, 8)
        ax.set_aspect('equal')
        
        # Plot limits (same as panel B)
        ax.set_xlim(-270, -170)  
        ax.set_ylim(-610, -510)
        
        # ROTATION FUNCTION
        def rotate_coords(x, y):
            """Rotate coordinates 90 degrees anticlockwise"""
            x_rot = -y
            y_rot = x
            return x_rot, y_rot
        
        # 1. Draw ROTATED harbor zones (same as panel B)
        for i, zone in enumerate(self.harbor_zones):
            zone_rotated = np.array([rotate_coords(p[0], p[1]) for p in zone])
            zone_patch = Polygon(zone_rotated, fc='lightgreen', ec='darkgreen', 
                            alpha=0.3, linewidth=2, linestyle='--')
            ax.add_patch(zone_patch)
            
            center = zone_rotated[:-1].mean(axis=0)
            ax.text(center[0], center[1], f'Zone {i+1}', 
                fontsize=11, weight='bold', ha='center', va='center',
                color='darkgreen', alpha=0.8)
        
        # 2. Draw ROTATED dock areas (same as panel B)
        for i, dock in enumerate(self.dock_areas):
            dock_rotated = np.array([rotate_coords(p[0], p[1]) for p in dock])
            dock_patch = Polygon(dock_rotated, fc='lightcoral', ec='darkred',
                            alpha=0.4, linewidth=2, hatch='///')
            ax.add_patch(dock_patch)
            
            center = dock_rotated[:-1].mean(axis=0)
            ax.text(center[0], center[1], f'Dock {i+1}',
                fontsize=11, weight='bold', ha='center', va='center', color='darkred', alpha=0.8)
        
        # 3. Rotate trajectory coordinates
        traj_x_rot, traj_y_rot = rotate_coords(trajectory['x'].values, trajectory['y'].values)
        
        # Find fault time index (15 seconds)
        fault_time = 15.0
        fault_idx = np.argmin(np.abs(trajectory['time'].values - fault_time))
        
        # Post-fault trajectory with RED gradient for failure
        points = np.array([traj_x_rot[fault_idx:], 
                        traj_y_rot[fault_idx:]]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        # Use RED color gradient for failure
        colors = plt.cm.Reds(np.linspace(0.3, 0.9, len(segments)))
        lc = LineCollection(segments, colors=colors, linewidth=3, alpha=0.8)
        ax.add_collection(lc)
        
        # Add drift indication with dashed line
        if len(trajectory) > fault_idx + 20:
            drift_points = np.array([traj_x_rot[-20:], 
                                    traj_y_rot[-20:]]).T.reshape(-1, 1, 2)
            drift_segments = np.concatenate([drift_points[:-1], drift_points[1:]], axis=1)
            drift_lc = LineCollection(drift_segments, colors='darkred', 
                                    linewidth=2, alpha=0.5, linestyles='dashed')
            ax.add_collection(drift_lc)
        
        # 4. Draw USV ghosts in RED
        mission_end_time = 125.5
        num_ghosts = 5
        ghost_times = np.linspace(fault_time, mission_end_time, num_ghosts)
        
        for i, t in enumerate(ghost_times):
            idx = np.argmin(np.abs(trajectory['time'] - t))
            if idx < len(trajectory):
                progress = (t - fault_time) / (mission_end_time - fault_time)
                alpha = 0.3 + 0.5 * progress
                
                # Rotate USV position
                usv_x_rot, usv_y_rot = rotate_coords(trajectory.iloc[idx]['x'], 
                                                    trajectory.iloc[idx]['y'])
                
                # Correct heading for rotation
                corrected_heading = trajectory.iloc[idx]['psi'] + np.pi/2
                
                time_since_fault = t - fault_time
                time_label = f"{time_since_fault:.0f}s"
                
                # Draw ship symbol in RED
                self.draw_ship_symbol(ax, usv_x_rot, usv_y_rot, corrected_heading,
                                    time_label, alpha=alpha, color='red')
        
        # 5. Add fault indicator (same as panel B)
        fault_x_rot, fault_y_rot = rotate_coords(trajectory.iloc[fault_idx]['x'],
                                                trajectory.iloc[fault_idx]['y'])
        
        ax.plot(fault_x_rot, fault_y_rot, 'r*', markersize=15, markeredgewidth=2,
            markeredgecolor='darkred', label='Fault Detected')
        
        ax.annotate('Left Thruster\n95% Loss', 
                xy=(fault_x_rot, fault_y_rot), 
                xytext=(fault_x_rot-22, fault_y_rot+15),
                fontsize=10, weight='bold', color='darkred',
                arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.3',
                                color='darkred', linewidth=2),
                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", 
                            edgecolor="darkred", alpha=0.8))
        
        # 6. Add FAILURE indicator (red X instead of green checkmark)
        final_x_rot, final_y_rot = rotate_coords(trajectory.iloc[-1]['x'],
                                                trajectory.iloc[-1]['y'])
        
        # Red X for failure
        cross_size = 6
        ax.plot([final_x_rot-cross_size, final_x_rot+cross_size], 
                [final_y_rot-cross_size, final_y_rot+cross_size], 
                'r-', linewidth=4, alpha=0.8)
        ax.plot([final_x_rot-cross_size, final_x_rot+cross_size], 
                [final_y_rot+cross_size, final_y_rot-cross_size], 
                'r-', linewidth=4, alpha=0.8)
        
        # Add "DRIFT" annotation
        ax.annotate('DRIFT', 
                xy=(final_x_rot, final_y_rot), 
                xytext=(final_x_rot+10, final_y_rot-12),
                fontsize=12, weight='bold', color='darkred',
                arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-0.3',
                                color='darkred', linewidth=2))
        
        # 7. Title and labels
        ax.set_xlabel('North (m)', fontsize=12, weight='bold')
        ax.set_ylabel('East (m)', fontsize=12, weight='bold')
        ax.set_title('Baseline MPC: Mission Failure Under Thruster Fault', 
                    fontsize=14, weight='bold', pad=20)
        
        # Mission FAILURE box (red instead of green)
        metrics_text = (
            f"✗ Mission Failed\n"
            f"USV Drifted Away\n"
            f"Unable to Reach Harbor"
            # f"Fault: Left Thruster 95% Degraded"
        )
        
        props = dict(boxstyle='round,pad=0.5', facecolor='lightcoral', 
                    alpha=0.9, edgecolor='darkred', linewidth=2)
        ax.text(0.95, 0.95, metrics_text, transform=ax.transAxes,
            fontsize=11, weight='bold', va='top', ha='right', bbox=props)
        
        # Environmental assistance INACTIVE indicator
        assist_text = "Virtual Actuators: INACTIVE"
        props = dict(boxstyle='round,pad=0.3', facecolor='lightgray',
                    alpha=0.9, edgecolor='darkgray', linewidth=2)
        ax.text(0.95, 0.05, assist_text, transform=ax.transAxes,
            fontsize=10, weight='bold', va='bottom', ha='right', 
            bbox=props, color='darkgray')
        
        # Grid and styling
        ax.grid(True, alpha=0.3, linestyle=':', linewidth=0.5)
        ax.tick_params(labelsize=10)
        ax.legend(loc='upper left', fontsize=10)
        
        # Scale bar
        ax.plot([-265, -245], [-606, -606], 'k-', linewidth=3)
        ax.text(-255, -605, '20 m', ha='center', fontsize=9)
        
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Panel A saved to {save_path}")
        
        return fig, ax

if __name__ == "__main__":
    # Usage - update with your baseline MPC (failure case) bag path
    bag_path = "/home/yang/usv_ws/experiments/bags/baseline_mpc_left_0.95_20250907_043919"  # Update this path
    
    generator = PanelAGenerator(bag_path)
    generator.read_bag()
    fig, ax = generator.generate_panel_a('icra_figure1_panel_a.pdf')
    
    plt.show()