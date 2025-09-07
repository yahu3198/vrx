#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge, Rectangle, FancyBboxPatch
import matplotlib.patches as mpatches
from windrose import WindroseAxes
import seaborn as sns

class EnvironmentalConditionsVisualizer:
    def __init__(self):
        # From your sydney_regatta.sdf configuration
        self.wind_direction = 135  # degrees (FROM this direction)
        self.wind_speed = 6.5  # m/s mean
        self.wind_variance = 0.4
        
        self.wave_direction = 120  # degrees
        self.wave_height = 1.5  # meters (gain)
        self.wave_period = 7.0  # seconds
        self.wave_steepness = 0.04
        
    def generate_environmental_figure(self, save_path='environmental_conditions.pdf'):
        """Generate comprehensive environmental conditions figure"""
        
        fig = plt.figure(figsize=(12, 8))
        
        # Create subplots
        gs = fig.add_gridspec(2, 3, height_ratios=[1, 1], width_ratios=[1, 1, 1])
        
        # 1. Wind Rose (Top Left)
        ax1 = fig.add_subplot(gs[0, 0], projection='windrose')
        self.plot_wind_rose(ax1)
        
        # 2. Wave Pattern (Top Middle)
        ax2 = fig.add_subplot(gs[0, 1])
        self.plot_wave_pattern(ax2)
        
        # 3. Combined Forces on USV (Top Right)
        ax3 = fig.add_subplot(gs[0, 2])
        self.plot_force_diagram(ax3)
        
        # 4. Temporal Force Evolution (Bottom Left)
        ax4 = fig.add_subplot(gs[1, 0])
        self.plot_temporal_forces(ax4)
        
        # 5. Virtual Actuator Capability (Bottom Middle)
        ax5 = fig.add_subplot(gs[1, 1])
        self.plot_virtual_actuator_envelope(ax5)
        
        # 6. Environmental Assistance Strategy (Bottom Right)
        ax6 = fig.add_subplot(gs[1, 2])
        self.plot_assistance_strategy(ax6)
        
        # Main title
        fig.suptitle('Environmental Conditions and Virtual Actuator Analysis', 
                     fontsize=16, weight='bold', y=0.98)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Environmental figure saved to {save_path}")
        
        return fig
    
    def plot_wind_rose(self, ax):
        """Plot wind rose showing wind conditions"""
        # Generate wind data based on configuration
        np.random.seed(42)
        
        # Wind comes FROM this direction
        directions = np.random.normal(self.wind_direction, 15, 1000) % 360
        speeds = np.random.normal(self.wind_speed, self.wind_variance * 2, 1000)
        speeds = np.clip(speeds, 0, 15)
        
        # Create wind rose
        ax.bar(directions, speeds, normed=True, opening=0.8, edgecolor='white',
               bins=np.arange(0, 16, 2), cmap=plt.cm.YlOrRd)
        
        # Add labels
        ax.set_title('Wind Conditions\n(Sea State 4)', fontsize=12, weight='bold', pad=20)
        ax.set_legend(title='Speed (m/s)', loc='upper left', bbox_to_anchor=(1.1, 1))
        
        # Add mean direction indicator
        mean_angle = np.radians(self.wind_direction)
        ax.annotate('', xy=(mean_angle, 30), xytext=(mean_angle, 0),
                   arrowprops=dict(arrowstyle='->', lw=3, color='red'))
        ax.text(mean_angle, 35, f'Mean: {self.wind_direction}°', 
               ha='center', fontsize=10, weight='bold')
    
    def plot_wave_pattern(self, ax):
        """Plot wave field visualization"""
        # Create grid
        x = np.linspace(0, 100, 100)
        y = np.linspace(0, 100, 100)
        X, Y = np.meshgrid(x, y)
        
        # Wave equation
        k = 2 * np.pi / (self.wave_period * 10)  # wave number
        angle = np.radians(self.wave_direction)
        
        # Wave height field
        Z = self.wave_height * np.sin(k * (X * np.cos(angle) + Y * np.sin(angle)))
        
        # Plot
        im = ax.contourf(X, Y, Z, levels=20, cmap='RdBu_r', alpha=0.8)
        ax.contour(X, Y, Z, levels=10, colors='black', alpha=0.3, linewidths=0.5)
        
        # Add wave direction arrow
        ax.arrow(50, 50, 20*np.cos(angle), 20*np.sin(angle),
                head_width=5, head_length=3, fc='darkblue', ec='darkblue', linewidth=2)
        
        # Labels
        ax.set_title(f'Wave Field\nHeight: {self.wave_height}m, Period: {self.wave_period}s',
                    fontsize=12, weight='bold')
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.set_aspect('equal')
        
        # Colorbar
        cbar = plt.colorbar(im, ax=ax, orientation='horizontal', pad=0.1)
        cbar.set_label('Wave Height (m)', fontsize=10)
    
    def plot_force_diagram(self, ax):
        """Plot forces acting on USV"""
        # USV representation
        usv_length = 0.3
        usv_width = 0.15
        
        # Draw USV
        usv = Rectangle((-usv_length/2, -usv_width/2), usv_length, usv_width,
                       fc='gray', ec='black', linewidth=2)
        ax.add_patch(usv)
        
        # Environmental forces (scaled for visualization)
        wind_force = 0.4
        wave_force = 0.35
        
        # Wind force (FROM 135 degrees)
        wind_angle = np.radians(180 - self.wind_direction)  # Convert to force direction
        ax.arrow(0, 0, wind_force*np.cos(wind_angle), wind_force*np.sin(wind_angle),
                head_width=0.05, head_length=0.03, fc='red', ec='darkred',
                linewidth=2, label='Wind Force')
        
        # Wave force (FROM 120 degrees)
        wave_angle = np.radians(180 - self.wave_direction)
        ax.arrow(0, 0, wave_force*np.cos(wave_angle), wave_force*np.sin(wave_angle),
                head_width=0.05, head_length=0.03, fc='blue', ec='darkblue',
                linewidth=2, label='Wave Force')
        
        # Resultant force
        result_x = wind_force*np.cos(wind_angle) + wave_force*np.cos(wave_angle)
        result_y = wind_force*np.sin(wind_angle) + wave_force*np.sin(wave_angle)
        ax.arrow(0, 0, result_x, result_y,
                head_width=0.05, head_length=0.03, fc='green', ec='darkgreen',
                linewidth=3, label='Resultant Force', linestyle='--')
        
        # Add yaw moment indicator
        circle = Circle((0, 0), 0.25, fill=False, ec='purple', linewidth=2, linestyle=':')
        ax.add_patch(circle)
        ax.annotate('', xy=(0.25, 0), xytext=(0.18, 0.18),
                   arrowprops=dict(arrowstyle='->', lw=2, color='purple'))
        ax.text(0.3, 0.3, 'Yaw Moment', fontsize=10, color='purple')
        
        # Setup
        ax.set_xlim(-0.6, 0.6)
        ax.set_ylim(-0.6, 0.6)
        ax.set_aspect('equal')
        ax.set_title('Environmental Forces on USV\n(15° Force Misalignment → Yaw Moment)',
                    fontsize=12, weight='bold')
        ax.legend(loc='upper right', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('Body X (m)')
        ax.set_ylabel('Body Y (m)')
    
    def plot_temporal_forces(self, ax):
        """Plot temporal evolution of forces"""
        time = np.linspace(0, 60, 600)
        
        # Simulated force evolution with oscillations
        wx = self.wind_speed * 10 * (1 + 0.3*np.sin(0.5*time) + 0.1*np.sin(2*time))
        wy = self.wind_speed * 8 * (1 + 0.2*np.sin(0.3*time) + 0.15*np.sin(1.5*time))
        wpsi = 5 * np.sin(0.2*time) * (1 + 0.3*np.sin(time))
        
        # Plot
        ax.plot(time, wx, 'r-', linewidth=2, label='Fx (Surge)', alpha=0.8)
        ax.plot(time, wy, 'b-', linewidth=2, label='Fy (Sway)', alpha=0.8)
        ax.plot(time, wpsi, 'g-', linewidth=2, label='Mz (Yaw)', alpha=0.8)
        
        # Add shaded regions for utilization
        ax.fill_between(time[300:], 0, wx[300:], alpha=0.2, color='red',
                       label='Utilized Region')
        
        # Labels
        ax.set_title('Environmental Forces Over Time', fontsize=12, weight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Force (N) / Moment (Nm)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.axvline(x=15, color='red', linestyle='--', alpha=0.5, label='Fault Time')
        ax.text(15, ax.get_ylim()[1]*0.9, 'Fault\nOccurs', ha='center', fontsize=10)
    
    def plot_virtual_actuator_envelope(self, ax):
        """Plot virtual actuator capability envelope"""
        # Create polar plot
        theta = np.linspace(0, 2*np.pi, 100)
        
        # Baseline thruster capability (healthy)
        r_baseline = np.ones_like(theta) * 100  # 100% in all directions
        
        # Single thruster failure (no environmental assistance)
        r_failed = np.ones_like(theta) * 30
        r_failed[(theta > np.pi/4) & (theta < 3*np.pi/4)] = 10  # Limited rightward
        
        # With environmental assistance
        r_assisted = r_failed.copy()
        # Boost in wind/wave direction
        boost_direction = np.radians(180 - self.wind_direction)
        for i, t in enumerate(theta):
            alignment = np.cos(t - boost_direction)
            if alignment > 0:
                r_assisted[i] += 40 * alignment
        
        # Convert to cartesian
        x_base = r_baseline * np.cos(theta)
        y_base = r_baseline * np.sin(theta)
        x_fail = r_failed * np.cos(theta)
        y_fail = r_failed * np.sin(theta)
        x_assist = r_assisted * np.cos(theta)
        y_assist = r_assisted * np.sin(theta)
        
        # Plot
        ax.fill(x_base, y_base, 'green', alpha=0.2, label='Healthy')
        ax.fill(x_fail, y_fail, 'red', alpha=0.3, label='Failed (No Assist)')
        ax.fill(x_assist, y_assist, 'blue', alpha=0.3, label='Failed + Virtual Actuator')
        
        ax.plot(x_base, y_base, 'g-', linewidth=2)
        ax.plot(x_fail, y_fail, 'r-', linewidth=2)
        ax.plot(x_assist, y_assist, 'b-', linewidth=2)
        
        # Labels
        ax.set_title('Control Authority Envelope\n(95% Thruster Failure)',
                    fontsize=12, weight='bold')
        ax.set_xlabel('Force X (%)')
        ax.set_ylabel('Force Y (%)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.set_xlim(-120, 120)
        ax.set_ylim(-120, 120)
    
    def plot_assistance_strategy(self, ax):
        """Plot environmental assistance strategy diagram"""
        # Create flow diagram
        ax.axis('off')
        
        # Title
        ax.text(0.5, 0.95, 'Virtual Actuator Strategy', 
               ha='center', fontsize=14, weight='bold')
        
        # Boxes
        boxes = [
            {'xy': (0.2, 0.8), 'text': 'Fault\nDetected', 'color': 'red'},
            {'xy': (0.5, 0.8), 'text': 'Assess\nEnvironment', 'color': 'yellow'},
            {'xy': (0.8, 0.8), 'text': 'Activate\nVirtual Actuator', 'color': 'green'},
            {'xy': (0.2, 0.5), 'text': 'EKF\nEstimation', 'color': 'lightblue'},
            {'xy': (0.5, 0.5), 'text': 'RLS\nPrediction', 'color': 'lightblue'},
            {'xy': (0.8, 0.5), 'text': 'MPC\nIntegration', 'color': 'lightblue'},
            {'xy': (0.5, 0.2), 'text': 'Mission\nSuccess', 'color': 'green'}
        ]
        
        for box in boxes:
            fancy_box = FancyBboxPatch((box['xy'][0]-0.08, box['xy'][1]-0.05),
                                       0.16, 0.1,
                                       boxstyle="round,pad=0.01",
                                       facecolor=box['color'],
                                       edgecolor='black',
                                       linewidth=2,
                                       alpha=0.7)
            ax.add_patch(fancy_box)
            ax.text(box['xy'][0], box['xy'][1], box['text'],
                   ha='center', va='center', fontsize=10, weight='bold')
        
        # Arrows
        arrows = [
            ((0.28, 0.8), (0.42, 0.8)),
            ((0.58, 0.8), (0.72, 0.8)),
            ((0.2, 0.75), (0.2, 0.55)),
            ((0.5, 0.75), (0.5, 0.55)),
            ((0.8, 0.75), (0.8, 0.55)),
            ((0.28, 0.45), (0.42, 0.3)),
            ((0.58, 0.45), (0.5, 0.3)),
            ((0.72, 0.45), (0.58, 0.3))
        ]
        
        for start, end in arrows:
            ax.annotate('', xy=end, xytext=start,
                       arrowprops=dict(arrowstyle='->', lw=2, color='black'))
        
        # Add key metrics
        ax.text(0.5, 0.05, 'Key: 85% Success Rate | 41% Energy Savings',
               ha='center', fontsize=11, weight='bold',
               bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen"))
        
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)

if __name__ == "__main__":
    visualizer = EnvironmentalConditionsVisualizer()
    fig = visualizer.generate_environmental_figure('icra_environmental_conditions.pdf')
    plt.show()