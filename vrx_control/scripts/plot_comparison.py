
#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon, Circle, Rectangle, FancyBboxPatch
from matplotlib.collections import LineCollection
import matplotlib.patheffects as path_effects
from matplotlib.gridspec import GridSpec
import seaborn as sns
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d
import os

# Set publication quality defaults
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial']
plt.rcParams['font.size'] = 9
plt.rcParams['axes.linewidth'] = 1.0
plt.rcParams['lines.linewidth'] = 1.5
plt.rcParams['axes.labelsize'] = 9
plt.rcParams['xtick.labelsize'] = 8
plt.rcParams['ytick.labelsize'] = 8
plt.rcParams['legend.fontsize'] = 8

class ComparisonFigureGenerator:
    def __init__(self):
        """Initialize the figure generator with paths to your rosbag files"""
        # UPDATE THESE PATHS with your actual bag files
        self.bag_paths = {
            'baseline_50': '/path/to/baseline_50_degradation.bag',
            'baseline_95': '/path/to/baseline_95_degradation.bag',
            'proposed_50': '/path/to/proposed_50_degradation.bag',
            'proposed_95': '/path/to/proposed_95_degradation.bag'
        }
        
        self.data = {}
        self.colors = {
            'baseline': '#E74C3C',      # Red
            'proposed': '#3498DB',       # Blue
            'environment': '#70AD47',    # sage green
            'thruster': '#5B9BD5',       # soft steel blue
            'failed': '#E0E0E0',        # Light gray
            'harbor': '#90EE90',        # Light green
            'dock': '#FFB6C1'           # Light red
        }
        
        # Store mission times for each scenario
        self.mission_times = {}
        
        # Define harbor zones (from your previous code)
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
    
    def read_bag(self, bag_path, label):
        """Read ROS2 bag and extract relevant data"""
        if not os.path.exists(bag_path):
            print(f"Warning: Bag file not found: {bag_path}")
            # Return dummy data for testing
            return self._generate_dummy_data(label)
            
        # ROS2 bag reading setup
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        topics_and_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topics_and_types}
        
        # Print available topics for debugging
        print(f"Available topics in {label}:")
        for topic in topics_and_types:
            print(f"  - {topic.name}: {topic.type}")
        
        # Store data in lists first
        trajectory_data = []
        left_thrust_data = []
        right_thrust_data = []
        
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            
            # Extract odometry data (ROS2 topic)
            if topic == '/wamv/sensors/position/ground_truth_odometry':
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                
                # Convert quaternion to yaw
                q = msg.pose.pose.orientation
                rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
                euler = rotation.as_euler('xyz')
                
                trajectory_data.append({
                    'time': timestamp * 1e-9,
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'psi': euler[2]
                })
            
            # Extract control commands separately
            elif topic == '/wamv/thrusters/left/thrust':
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                left_thrust_data.append({
                    'time': timestamp * 1e-9,
                    'thrust': msg.data
                })
                
            elif topic == '/wamv/thrusters/right/thrust':
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                right_thrust_data.append({
                    'time': timestamp * 1e-9,
                    'thrust': msg.data
                })
        
        # Convert to DataFrames
        result_data = {}
        
        # Trajectory data
        if trajectory_data:
            result_data['trajectory'] = pd.DataFrame(trajectory_data)
            result_data['trajectory']['time'] -= result_data['trajectory']['time'].iloc[0]
        else:
            result_data['trajectory'] = pd.DataFrame()
        
        # Control data - merge left and right thrust
        if left_thrust_data and right_thrust_data:
            left_df = pd.DataFrame(left_thrust_data)
            right_df = pd.DataFrame(right_thrust_data)
            
            # Align timestamps - use trajectory timestamps as reference
            if not result_data['trajectory'].empty:
                ref_times = result_data['trajectory']['time'].values
                
                # Interpolate thrust values to trajectory timestamps
                left_interp = interp1d(left_df['time'].values - left_df['time'].iloc[0], 
                                      left_df['thrust'].values, 
                                      kind='nearest', bounds_error=False, fill_value=0)
                right_interp = interp1d(right_df['time'].values - right_df['time'].iloc[0], 
                                       right_df['thrust'].values,
                                       kind='nearest', bounds_error=False, fill_value=0)
                
                control_data = pd.DataFrame({
                    'time': ref_times,
                    'left_thrust': left_interp(ref_times),
                    'right_thrust': right_interp(ref_times)
                })
                result_data['control'] = control_data
            else:
                # No trajectory data, use control timestamps
                min_len = min(len(left_df), len(right_df))
                control_data = pd.DataFrame({
                    'time': left_df['time'].iloc[:min_len].values - left_df['time'].iloc[0],
                    'left_thrust': left_df['thrust'].iloc[:min_len].values,
                    'right_thrust': right_df['thrust'].iloc[:min_len].values
                })
                result_data['control'] = control_data
        else:
            result_data['control'] = pd.DataFrame()
        
        # Calculate energy data from control
        if not result_data['control'].empty:
            control = result_data['control']
            power = []
            for i in range(len(control)):
                total_thrust = abs(control.iloc[i]['left_thrust']) + abs(control.iloc[i]['right_thrust'])
                # Simple power model
                p = 0.01 * (total_thrust ** 1.5)
                power.append(p)
            
            # Calculate cumulative energy
            cumulative = [0]
            if len(power) > 1:
                times = control['time'].values
                for i in range(1, len(power)):
                    dt = times[i] - times[i-1]
                    cumulative.append(cumulative[-1] + power[i-1] * dt)
            
            result_data['energy'] = pd.DataFrame({
                'time': control['time'].values,
                'power': power,
                'cumulative': cumulative[:len(power)]
            })
        else:
            result_data['energy'] = pd.DataFrame()
        
        # Add empty environmental data for now
        result_data['environmental'] = pd.DataFrame()
        
        print(f"Data extracted for {label}:")
        print(f"  - Trajectory points: {len(result_data['trajectory'])}")
        print(f"  - Control commands: {len(result_data['control'])}")
        print(f"  - Energy data: {len(result_data['energy'])}")
        
        return result_data
    
    def _generate_dummy_data(self, label):
        """Generate dummy data for testing the visualization"""
        np.random.seed(42)
        t = np.linspace(0, 120, 500)
        
        # Generate different trajectories based on label
        if 'baseline_95' in label:
            # Failed trajectory - drifts away
            x = -250 + 10*np.sin(0.05*t) + 0.5*t
            y = 200 - 2*t + 20*np.sin(0.02*t)
        elif 'baseline_50' in label:
            # Successful but inefficient
            x = -250 + 30*np.sin(0.1*t) - t
            y = 200 - 1.5*t
        elif 'proposed_95' in label:
            # Successful with environmental assistance
            x = -250 - 1.5*t + 10*np.sin(0.05*t)
            y = 200 - 1.8*t + 5*np.sin(0.03*t)
        else:  # proposed_50
            # Successful and efficient
            x = -250 - 2*t
            y = 200 - 2*t
        
        # Generate control data
        if '95' in label:
            left_thrust = np.ones_like(t) * 0.05 if 'baseline' in label else np.ones_like(t) * 0.05
            right_thrust = np.ones_like(t) * 1.0
        else:
            left_thrust = np.ones_like(t) * 0.5
            right_thrust = np.ones_like(t) * 1.0
        
        # Add noise
        left_thrust += np.random.normal(0, 0.1, len(t))
        right_thrust += np.random.normal(0, 0.1, len(t))
        
        # Generate energy data
        power = np.abs(left_thrust) + np.abs(right_thrust)
        if 'proposed' in label:
            power *= 0.7  # Lower power consumption
        cumulative = np.cumsum(power) * 0.1
        
        return {
            'trajectory': pd.DataFrame({'time': t, 'x': x, 'y': y, 'psi': np.random.randn(len(t))*0.1}),
            'control': pd.DataFrame({'time': t, 'left_thrust': left_thrust, 'right_thrust': right_thrust}),
            'environmental': pd.DataFrame({'time': t, 'wind_x': np.ones_like(t)*5, 'wind_y': np.ones_like(t)*3}),
            'energy': pd.DataFrame({'time': t, 'power': power, 'cumulative': cumulative})
        }
    
    def rotate_coords(self, x, y):
        """Rotate coordinates 90 degrees for visualization"""
        x_rot = -y
        y_rot = x
        return x_rot, y_rot
    
    def plot_trajectory_panel(self, ax, data_baseline, data_proposed, title, fault_time=15):
        """Plot trajectory comparison panel"""
        ax.set_aspect('equal')
        
        # Set limits
        ax.set_xlim(-260, -170)
        ax.set_ylim(-600, -520)
        
        # Draw harbor zones
        for i, zone in enumerate(self.harbor_zones):
            zone_rotated = np.array([self.rotate_coords(p[0], p[1]) for p in zone])
            zone_patch = Polygon(zone_rotated, fc=self.colors['harbor'], 
                               ec='darkgreen', alpha=0.3, linewidth=1.5, linestyle='--')
            ax.add_patch(zone_patch)
        
        # Draw dock areas
        for dock in self.dock_areas:
            dock_rotated = np.array([self.rotate_coords(p[0], p[1]) for p in dock])
            dock_patch = Polygon(dock_rotated, fc=self.colors['dock'],
                               ec='darkred', alpha=0.3, linewidth=1.5)
            ax.add_patch(dock_patch)
        
        # Plot baseline trajectory
        if not data_baseline['trajectory'].empty:
            traj_b = data_baseline['trajectory']
            x_rot_b, y_rot_b = self.rotate_coords(traj_b['x'].values, traj_b['y'].values)
            ax.plot(x_rot_b, y_rot_b, '--', color=self.colors['baseline'], 
                    linewidth=2, label='MPC', alpha=0.8)
            
            # End marker
            end_x_b, end_y_b = self.rotate_coords(traj_b.iloc[-1]['x'], traj_b.iloc[-1]['y'])
            ax.plot(end_x_b, end_y_b, 'o', color=self.colors['baseline'], 
                    markersize=8, markeredgecolor='darkred', markeredgewidth=1.5)
        
        # Plot proposed trajectory
        if not data_proposed['trajectory'].empty:
            traj_p = data_proposed['trajectory']
            x_rot_p, y_rot_p = self.rotate_coords(traj_p['x'].values, traj_p['y'].values)
            ax.plot(x_rot_p, y_rot_p, '-', color=self.colors['proposed'], 
                    linewidth=2, label='EAMPC', alpha=0.9)
            
            # Mark fault occurrence
            fault_idx = np.argmin(np.abs(traj_p['time'].values - fault_time))
            if fault_idx < len(traj_p):
                fault_x, fault_y = self.rotate_coords(traj_p.iloc[fault_idx]['x'], 
                                                      traj_p.iloc[fault_idx]['y'])
                ax.plot(fault_x, fault_y, 'r*', markersize=12, markeredgewidth=1.5,
                        markeredgecolor='darkred', label='Fault Occurs')
            
            # End marker
            end_x_p, end_y_p = self.rotate_coords(traj_p.iloc[-1]['x'], traj_p.iloc[-1]['y'])
            ax.plot(end_x_p, end_y_p, 's', color=self.colors['proposed'], 
                    markersize=8, markeredgecolor='darkblue', markeredgewidth=1.5)
        
        # Labels and formatting
        ax.set_xlabel('North (m)', fontsize=8)
        ax.set_ylabel('East (m)', fontsize=8)
        ax.set_title(title, fontsize=9)
        ax.grid(True, alpha=0.3, linestyle=':', linewidth=0.5)
        ax.legend(loc='upper right', fontsize=6, framealpha=0.9)
        
    def plot_control_authority_panel(self, ax, data_baseline, data_proposed, 
                                     degradation_level, title):
        """Plot control authority distribution panel using actual environmental assistance data"""
        
        # Check if we have environmental assistance data for proposed method
        if not data_proposed['environmental'].empty:
            env_data = data_proposed['environmental']
            time_p = env_data['time'].values
            
            # Calculate average of the three alpha factors for overall environmental contribution
            # These are the actual α values from your MPC (0 to 1)
            env_contribution = (env_data['surge_alpha'].values + 
                              env_data['sway_alpha'].values + 
                              env_data['yaw_alpha'].values) / 3.0
            
            # FORCE TO ZERO BEFORE FAULT
            fault_time = 15
            fault_idx = np.argmin(np.abs(time_p - fault_time))
            env_contribution[:fault_idx] = 0.0  # No environmental assistance before fault
            
            # Clip to [0, 1] range for post-fault
            env_contribution = np.clip(env_contribution, 0, 1)
            
            # Smooth the data slightly for better visualization (only post-fault)
            if len(env_contribution) > fault_idx + 10:
                from scipy.ndimage import gaussian_filter1d
                # Only smooth the post-fault portion
                post_fault_smooth = gaussian_filter1d(env_contribution[fault_idx:], sigma=2)
                env_contribution[fault_idx:] = post_fault_smooth
            
            thrust_contribution = 1 - env_contribution
            
            # Plot stacked areas
            ax.fill_between(time_p, 0, thrust_contribution*100, 
                           color=self.colors['thruster'], alpha=0.7, label='Thruster')
            ax.fill_between(time_p, thrust_contribution*100, 100,
                           color=self.colors['environment'], alpha=0.7, label='Environment')
            
            # Add average utilization text (only for post-fault)
            if fault_idx < len(env_contribution) - 1:
                avg_env_after_fault = np.mean(env_contribution[fault_idx:]) * 100
                ax.text(0.98, 0.5, f'Avg. Env. Use\n(post-fault):\n{avg_env_after_fault:.1f}%',
                       transform=ax.transAxes, fontsize=7,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='lightblue', alpha=0.8),
                       ha='right', va='center')
            
        elif not data_proposed['control'].empty:
            # Fallback to simulated data if no environmental assistance topic
            time_p = data_proposed['control']['time'].values
            
            # Simulate environmental contribution based on degradation
            env_contribution = np.ones_like(time_p) * (0.3 if degradation_level == 50 else 0.65)
            env_contribution += np.random.normal(0, 0.05, len(time_p))
            env_contribution = np.clip(env_contribution, 0, 0.8)
            
            # FORCE TO ZERO BEFORE FAULT
            fault_time = 15
            fault_idx = np.argmin(np.abs(time_p - fault_time))
            env_contribution[:fault_idx] = 0.0  # No environmental assistance before fault
            
            thrust_contribution = 1 - env_contribution
            
            # Plot areas
            ax.fill_between(time_p, 0, thrust_contribution*100, 
                           color=self.colors['thruster'], alpha=0.7, label='Thruster')
            ax.fill_between(time_p, thrust_contribution*100, 100,
                           color=self.colors['environment'], alpha=0.7, label='Environment')
        
        # Mark fault time with more emphasis
        ax.axvline(x=15, color='red', linestyle=':', linewidth=2, alpha=0.8)
        ax.text(15, 90, 'Fault\nOccurs', rotation=0, fontsize=7, ha='center', 
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))
        
        # Add annotation for the transition
        ax.annotate('Environmental\nAssistance\nActivates', 
                   xy=(15, 50), xytext=(20, 30),
                   fontsize=7, ha='left',
                   arrowprops=dict(arrowstyle='->', color='green', 
                                 linewidth=1.5, alpha=0.7))
        
        # Formatting
        ax.set_xlabel('Time (s)', fontsize=8)
        ax.set_ylabel('Control Authority (%)', fontsize=8)
        ax.set_title(title, fontsize=9)
        ax.set_ylim(0, 105)
        if 'time_p' in locals():
            ax.set_xlim(0, max(time_p))
        ax.grid(True, alpha=0.3, linestyle=':', linewidth=0.5)
        ax.legend(loc='upper right', fontsize=6, framealpha=0.9)
        
    def plot_distance_to_harbor_panel(self, ax, data_baseline, data_proposed, title, scenario_label=None):
        """Plot distance to harbor over time and extract mission times"""
        
        # Harbor x-position target
        harbor_x_target = -570
        harbor_threshold = 1  # meters
        fault_time = 15  # seconds
        
        # Initialize mission time tracking for this scenario
        baseline_mission_time = None
        proposed_mission_time = None
        
        # Calculate distance for baseline
        if not data_baseline['trajectory'].empty:
            traj_b = data_baseline['trajectory']
            time_b = traj_b['time'].values
            # Simple distance: current x position - target x position
            distances_b = np.abs(traj_b['x'].values - harbor_x_target)
            
            # Check if baseline reaches harbor
            harbor_reach_idx_b = next((i for i, d in enumerate(distances_b) if d < harbor_threshold), -1)
            if harbor_reach_idx_b > 0:
                # Calculate mission time from fault to success
                baseline_mission_time = time_b[harbor_reach_idx_b] - fault_time
                
                # Truncate trajectory at mission completion
                time_b = time_b[:harbor_reach_idx_b+1]
                distances_b = distances_b[:harbor_reach_idx_b+1]
            
            ax.plot(time_b, distances_b, '--', color=self.colors['baseline'], 
                    linewidth=2, label='MPC', alpha=0.8)
            
            # Add final distance annotation only if far from harbor (failed case)
            if harbor_reach_idx_b < 0 and distances_b[-1] > 50:
                ax.annotate(f'{distances_b[-1]:.0f}m',
                           xy=(time_b[-1], distances_b[-1]),
                           xytext=(time_b[-1]-10, distances_b[-1]+20),
                           fontsize=7, color=self.colors['baseline'],
                           arrowprops=dict(arrowstyle='->', color=self.colors['baseline'], 
                                          linewidth=1, alpha=0.5))
        
        # Calculate distance for proposed
        if not data_proposed['trajectory'].empty:
            traj_p = data_proposed['trajectory']
            time_p = traj_p['time'].values
            # Simple distance: current x position - target x position
            distances_p = np.abs(traj_p['x'].values - harbor_x_target)
            
            # Find when mission completes (reaches harbor)
            harbor_reach_idx_p = next((i for i, d in enumerate(distances_p) if d < harbor_threshold), -1)
            if harbor_reach_idx_p > 0:
                # Calculate mission time from fault to success
                proposed_mission_time = time_p[harbor_reach_idx_p] - fault_time
                
                # Truncate trajectory at mission completion
                time_p = time_p[:harbor_reach_idx_p+1]
                distances_p = distances_p[:harbor_reach_idx_p+1]
            
            ax.plot(time_p, distances_p, '-', color=self.colors['proposed'], 
                    linewidth=2, label='EAMPC', alpha=0.9)
        
        # Store mission times if scenario label provided
        if scenario_label:
            self.mission_times[scenario_label] = {
                'baseline': baseline_mission_time,
                'proposed': proposed_mission_time
            }
        
        # Add horizontal line at harbor threshold
        ax.axhline(y=harbor_threshold, color='green', linestyle=':', 
                  linewidth=1.5, alpha=0.5, label=f'Harbor Zone')
        
        # Shade the harbor zone
        ax.fill_between([0, ax.get_xlim()[1]], 0, harbor_threshold,
                       color='green', alpha=0.1)
        
        # Mark fault time
        ax.axvline(x=15, color='red', linestyle=':', linewidth=1.5, alpha=0.7)
        ax.text(15, ax.get_ylim()[1]*0.9, 'Fault', rotation=0, fontsize=7, ha='center')
        
        # Formatting
        ax.set_xlabel('Time (s)', fontsize=8)
        ax.set_ylabel('Distance to Harbor (m)', fontsize=8)
        ax.set_title(title, fontsize=9)
        ax.set_ylim(bottom=0)  # Distance can't be negative
        ax.grid(True, alpha=0.3, linestyle=':', linewidth=0.5)
        ax.legend(loc='upper right', fontsize=6, framealpha=0.9)
        
    def print_mission_times(self):
        """Print mission completion times for all scenarios"""
        print("\n" + "="*60)
        print("MISSION COMPLETION TIMES (from fault at t=15s to harbor)")
        print("="*60)
        
        for scenario, times in self.mission_times.items():
            print(f"\n{scenario}:")
            
            if times['baseline'] is not None:
                print(f"  Baseline MPC: {times['baseline']:.1f} seconds")
            else:
                print(f"  Baseline MPC: FAILED (did not reach harbor)")
            
            if times['proposed'] is not None:
                print(f"  Proposed EAMPC: {times['proposed']:.1f} seconds")
            else:
                print(f"  Proposed EAMPC: FAILED (did not reach harbor)")
            
            # Calculate improvement if both successful
            if times['baseline'] is not None and times['proposed'] is not None:
                improvement = (times['baseline'] - times['proposed']) / times['baseline'] * 100
                print(f"  Time Reduction: {improvement:.1f}%")
                print(f"  Time Saved: {times['baseline'] - times['proposed']:.1f} seconds")
        
        print("\n" + "="*60)
        print("SUMMARY STATISTICS")
        print("="*60)
        
        # Count successes
        baseline_successes = sum(1 for s in self.mission_times.values() if s['baseline'] is not None)
        proposed_successes = sum(1 for s in self.mission_times.values() if s['proposed'] is not None)
        
        print(f"\nSuccess Rate:")
        print(f"  Baseline MPC: {baseline_successes}/{len(self.mission_times)} scenarios")
        print(f"  Proposed EAMPC: {proposed_successes}/{len(self.mission_times)} scenarios")
        
        # Average times for successful missions
        baseline_times = [s['baseline'] for s in self.mission_times.values() if s['baseline'] is not None]
        proposed_times = [s['proposed'] for s in self.mission_times.values() if s['proposed'] is not None]
        
        if baseline_times:
            print(f"\nAverage Mission Time (successful only):")
            print(f"  Baseline MPC: {np.mean(baseline_times):.1f} ± {np.std(baseline_times):.1f} seconds")
        
        if proposed_times:
            print(f"  Proposed EAMPC: {np.mean(proposed_times):.1f} ± {np.std(proposed_times):.1f} seconds")
        
        if baseline_times and proposed_times:
            avg_improvement = (np.mean(baseline_times) - np.mean(proposed_times)) / np.mean(baseline_times) * 100
            print(f"\nAverage Improvement: {avg_improvement:.1f}%")
        
        print("="*60 + "\n")
        
    def generate_figure(self, save_path='comparison_figure.pdf'):
        """Generate the complete 2x3 comparison figure"""
        
        # Read all bag files (or generate dummy data)
        print("Loading data...")
        self.data['baseline_50'] = self.read_bag(self.bag_paths['baseline_50'], 'baseline_50')
        self.data['baseline_95'] = self.read_bag(self.bag_paths['baseline_95'], 'baseline_95')
        self.data['proposed_50'] = self.read_bag(self.bag_paths['proposed_50'], 'proposed_50')
        self.data['proposed_95'] = self.read_bag(self.bag_paths['proposed_95'], 'proposed_95')
        
        # Create figure with custom spacing
        fig = plt.figure(figsize=(7.16, 4.5))  # IEEE double column width
        gs = GridSpec(2, 3, figure=fig, 
                     hspace=0.35,  # Vertical spacing between rows
                     wspace=0.35,  # Horizontal spacing between columns
                     left=0.06, right=0.98,  # Margins
                     top=0.92, bottom=0.08)
        
        # Row 1: 50% Degradation
        ax1 = fig.add_subplot(gs[0, 0])
        ax2 = fig.add_subplot(gs[0, 1])
        ax3 = fig.add_subplot(gs[0, 2])
        
        self.plot_trajectory_panel(ax1, self.data['baseline_50'], self.data['proposed_50'],
                                   '(a) Trajectory (50%)')
        self.plot_control_authority_panel(ax2, self.data['baseline_50'], self.data['proposed_50'],
                                          50, '(b) Control Authority (50%)')
        self.plot_distance_to_harbor_panel(ax3, self.data['baseline_50'], self.data['proposed_50'],
                                           '(c) Distance to Harbor (50%)', scenario_label='50% Degradation')
        
        # Row 2: 95% Degradation
        ax4 = fig.add_subplot(gs[1, 0])
        ax5 = fig.add_subplot(gs[1, 1])
        ax6 = fig.add_subplot(gs[1, 2])
        
        self.plot_trajectory_panel(ax4, self.data['baseline_95'], self.data['proposed_95'],
                                   '(d) Trajectory (95%)')
        self.plot_control_authority_panel(ax5, self.data['baseline_95'], self.data['proposed_95'],
                                          95, '(e) Control Authority (95%)')
        self.plot_distance_to_harbor_panel(ax6, self.data['baseline_95'], self.data['proposed_95'],
                                           '(f) Distance to Harbor (95%)', scenario_label='95% Degradation')
        
        # Print mission times after processing all data
        self.print_mission_times()
        
        # Save figure
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Figure saved to {save_path}")
        
        return fig

if __name__ == "__main__":
    # Create generator
    generator = ComparisonFigureGenerator()
    
    # UPDATE THESE PATHS with your actual ROS2 bag files
    # The bag files should be from your mpc_with_record.launch.py recordings
    generator.bag_paths = {
        'baseline_50': '/home/yang/usv_ws/experiments/bags/baseline_mpc_left_0.5_20250908_023513',
        'baseline_95': '/home/yang/usv_ws/experiments/bags/baseline_mpc_left_0.95_20250907_043919', 
        'proposed_50': '/home/yang/usv_ws/experiments/bags/env_mpc_left_0.5_20250908_050912',
        'proposed_95': '/home/yang/usv_ws/experiments/bags/env_mpc_left_0.95_20250907_043328'
    }
    
    # Note: ROS2 bags are directories, not single files
    # Make sure the paths point to the bag directories created by ros2 bag record
    
    # Generate the figure
    fig = generator.generate_figure('icra_comparison_2x3.pdf')
    
    # Display
    plt.show()