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

class ICRAFigureGenerator:
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
    
    def extract_environmental_forces(self):
        """Extract environmental forces being utilized"""
        env_data = self.data.get('/wamv/environmental_assistance', {})
        forces = {'time': [], 'surge_factor': [], 'sway_factor': [], 'yaw_factor': [],
                  'wx': [], 'wy': [], 'wpsi': []}
        
        for i, msg in enumerate(env_data.get('messages', [])):
            forces['time'].append(env_data['timestamps'][i] - env_data['timestamps'][0])
            forces['surge_factor'].append(msg.data[0])
            forces['sway_factor'].append(msg.data[1])
            forces['yaw_factor'].append(msg.data[2])
            forces['wx'].append(msg.data[3])
            forces['wy'].append(msg.data[4])
            forces['wpsi'].append(msg.data[5])
        
        return pd.DataFrame(forces)
    
    def extract_metrics(self):
        """Extract mission metrics"""
        metrics_data = self.data.get('/wamv/mission_metrics', {})
        
        if metrics_data and len(metrics_data['messages']) > 0:
            last_msg = metrics_data['messages'][-1]
            return {
                'duration': last_msg.data[0],
                'energy': last_msg.data[1] / 1000.0,  # Convert to kJ
                'success': last_msg.data[4] > 0.5,
                'final_zone': int(last_msg.data[5])
            }
        return None
    
    def draw_usv_wedge(self, ax, x, y, heading, time_label, alpha=0.8, color='green'):
        """Draw stylized USV as wedge shape"""
        # USV dimensions (scaled for visibility)
        length = 8
        width = 5
        
        # Create wedge shape
        wedge = Wedge((x, y), length, 
                      np.degrees(heading - np.pi/6), 
                      np.degrees(heading + np.pi/6),
                      fc=color, ec='darkgreen', alpha=alpha, linewidth=1.5)
        ax.add_patch(wedge)
        
        # Add heading indicator
        dx = length * 1.2 * np.cos(heading)
        dy = length * 1.2 * np.sin(heading)
        ax.arrow(x, y, dx, dy, head_width=2, head_length=2, 
                fc=color, ec='darkgreen', alpha=alpha*0.8, linewidth=1)
        
        # Add time label
        if time_label is not None:
            text = ax.text(x + 10, y + 10, f't={time_label}s',
                          fontsize=8, alpha=alpha*0.7, weight='bold')
            text.set_path_effects([path_effects.withStroke(linewidth=3, foreground='white')])
    
    def generate_panel_b(self, save_path='figure1_panel_b.pdf'):
        """Generate Panel B showing successful environment-assisted return"""
        
        # Extract data
        trajectory = self.extract_trajectory()
        env_forces = self.extract_environmental_forces()
        metrics = self.extract_metrics()
        
        # Create figure
        fig, ax = plt.subplots(figsize=(10, 6), dpi=150)  # Was (8, 8)
        ax.set_aspect('equal')
        
        # CORRECTED plot limits - moved scene DOWN (towards more negative east values)
        ax.set_xlim(-270, -170)  
        ax.set_ylim(-610, -510)  # More negative = downwards in the plot
        
        # ROTATION FUNCTION
        def rotate_coords(x, y):
            """Rotate coordinates 90 degrees anticlockwise"""
            x_rot = -y
            y_rot = x
            return x_rot, y_rot
        
        # 1. Draw ROTATED harbor zones
        for i, zone in enumerate(self.harbor_zones):
            zone_rotated = np.array([rotate_coords(p[0], p[1]) for p in zone])
            zone_patch = Polygon(zone_rotated, fc='lightgreen', ec='darkgreen', 
                            alpha=0.3, linewidth=2, linestyle='--')
            ax.add_patch(zone_patch)
            
            center = zone_rotated[:-1].mean(axis=0)
            ax.text(center[0], center[1], f'Zone {i+1}', 
                fontsize=11, weight='bold', ha='center', va='center',
                color='darkgreen', alpha=0.8)
        
        # 2. Draw ROTATED dock areas
        for i, dock in enumerate(self.dock_areas):
            dock_rotated = np.array([rotate_coords(p[0], p[1]) for p in dock])
            dock_patch = Polygon(dock_rotated, fc='lightcoral', ec='darkred',
                            alpha=0.4, linewidth=2, hatch='///')
            ax.add_patch(dock_patch)
            
            center = dock_rotated[:-1].mean(axis=0)
            ax.text(center[0], center[1], f'Dock {i+1}',
                fontsize=9, ha='center', va='center', color='darkred', alpha=0.8)
        
        # 3. Rotate trajectory coordinates
        traj_x_rot, traj_y_rot = rotate_coords(trajectory['x'].values, trajectory['y'].values)
        
        # Find fault time index (15 seconds)
        fault_time = 15.0
        fault_idx = np.argmin(np.abs(trajectory['time'].values - fault_time))
        
        # Post-fault trajectory with gradient
        points = np.array([traj_x_rot[fault_idx:], 
                        traj_y_rot[fault_idx:]]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        colors = plt.cm.YlGn(np.linspace(0.3, 0.9, len(segments)))
        lc = LineCollection(segments, colors=colors, linewidth=3, alpha=0.8)
        ax.add_collection(lc)
        
        # 4. Draw SIMPLIFIED USV ghosts (ship symbols)
        mission_end_time = trajectory['time'].values[-1]
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
                
                # Draw simplified ship symbol in BLUE
                self.draw_ship_symbol(ax, usv_x_rot, usv_y_rot, corrected_heading,
                                    time_label, alpha=alpha, color='blue')  # Changed to blue
        
        # 5. Add fault indicator
        fault_x_rot, fault_y_rot = rotate_coords(trajectory.iloc[fault_idx]['x'],
                                                trajectory.iloc[fault_idx]['y'])
        
        ax.plot(fault_x_rot, fault_y_rot, 'r*', markersize=15, markeredgewidth=2,
            markeredgecolor='darkred', label='Fault Detected')
        
        ax.annotate('Left Thruster\n95% Loss', 
                xy=(fault_x_rot, fault_y_rot), 
                xytext=(fault_x_rot-25, fault_y_rot+15),
                fontsize=10, weight='bold', color='darkred',
                arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.3',
                                color='darkred', linewidth=2),
                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", 
                            edgecolor="darkred", alpha=0.8))
        
        # 6. Add success indicator
        final_x_rot, final_y_rot = rotate_coords(trajectory.iloc[-1]['x'],
                                                trajectory.iloc[-1]['y'])
        
        # Success checkmark
        ax.plot([final_x_rot-3, final_x_rot], [final_y_rot-3, final_y_rot-6], 'g-', linewidth=4)
        ax.plot([final_x_rot, final_x_rot+5], [final_y_rot-6, final_y_rot+2], 'g-', linewidth=4)
        
        # 7. Title and labels
        ax.set_xlabel('North (m)', fontsize=12, weight='bold')
        ax.set_ylabel('East (m)', fontsize=12, weight='bold')
        ax.set_title('Environment-Assisted MPC: Successful Harbor Return', 
                    fontsize=14, weight='bold', pad=20)
        
        # Metrics box
        if metrics:
            recovery_time = mission_end_time - fault_time
            metrics_text = (
                f"✓ Mission Success\n"
                f"Mission Duration: {recovery_time:.1f}s\n"
                f"Energy Used: {metrics['energy']:.1f} kJ"
                # f"Fault: Left Thruster 95% Degraded"
            )
            
            props = dict(boxstyle='round,pad=0.5', facecolor='lightgreen', 
                        alpha=0.9, edgecolor='darkgreen', linewidth=2)
            ax.text(0.95, 0.95, metrics_text, transform=ax.transAxes,
                fontsize=11, weight='bold', va='top', ha='right', bbox=props)
        
        # Environmental assistance indicator
        assist_text = "Virtual Actuators: ACTIVE"
        props = dict(boxstyle='round,pad=0.3', facecolor='lightblue',
                    alpha=0.9, edgecolor='darkblue', linewidth=2)
        ax.text(0.95, 0.05, assist_text, transform=ax.transAxes,
            fontsize=10, weight='bold', va='bottom', ha='right', 
            bbox=props, color='darkblue')
        
        # Grid and styling
        ax.grid(True, alpha=0.3, linestyle=':', linewidth=0.5)
        ax.tick_params(labelsize=10)
        ax.legend(loc='upper left', fontsize=10)
        
        # Scale bar (adjusted position)
        ax.plot([-265, -245], [-606, -606], 'k-', linewidth=3)
        ax.text(-255, -605, '20 m', ha='center', fontsize=9)
        
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Figure saved to {save_path}")
        
        return fig, ax

    def draw_ship_symbol(self, ax, x, y, heading, time_label, alpha=0.8, color='blue'):
        """Draw simplified ship symbol (rectangle + triangle) in blue"""
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
        
        # Draw ship in BLUE
        ship_patch = Polygon(rotated_vertices[:-1], fc='dodgerblue', ec='steelblue', 
                    alpha=alpha, linewidth=1.5)
        ax.add_patch(ship_patch)
        
        # Add time label CLOSER to ship
        if time_label is not None:
            # Reduced offset from 12 to 5 for closer positioning
            text = ax.text(x + 0.6, y + 2, time_label,
                        fontsize=8, alpha=alpha*0.7, weight='bold')
            text.set_path_effects([path_effects.withStroke(linewidth=3, foreground='white')])
        

if __name__ == "__main__":
    # Usage
    bag_path = "/home/yang/usv_ws/experiments/bags/env_mpc_left_0.95_20250907_043328"  # Update with your bag path
    
    generator = ICRAFigureGenerator(bag_path)
    generator.read_bag()
    fig, ax = generator.generate_panel_b('icra_figure1_panel_b.pdf')

    
    
    plt.show()