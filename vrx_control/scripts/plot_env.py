#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon
from matplotlib.patches import FancyBboxPatch
import matplotlib.patches as patches

# Set publication quality defaults - smaller fonts for compact figure
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial']
plt.rcParams['font.size'] = 7
plt.rcParams['axes.linewidth'] = 0.8
plt.rcParams['lines.linewidth'] = 1.2
plt.rcParams['axes.labelsize'] = 7
plt.rcParams['xtick.labelsize'] = 6
plt.rcParams['ytick.labelsize'] = 6
plt.rcParams['legend.fontsize'] = 6

class EnvironmentalForcesPlot:
    def __init__(self):
        # Environmental parameters from sydney_regatta.sdf
        self.wind_direction = 135  # degrees (FROM this direction)
        self.wind_speed = 6.5  # m/s
        self.wave_direction = 120  # degrees  
        self.wave_height = 1.5  # meters
        self.wave_period = 7.0  # seconds
        
    def generate_figure(self, save_path='environmental_forces_compact.pdf'):
        """Generate compact environmental forces figure"""
        
        # Create figure - 2/3 of original size (original was part of 3.5" wide)
        # Original panel was roughly 3.5" x 2.5", so 2/3 would be ~2.3" x 1.7"
        fig, ax = plt.subplots(figsize=(2.3, 1.7))
        ax.set_aspect('equal')
        
        # Smaller plot area (2/3 of original range)
        ax.set_xlim(-233, -207)  # Originally -240 to -200, now 26 units wide
        ax.set_ylim(-575, -555)  # Originally -585 to -545, now 20 units tall
        
        # Simple wave background
        x = np.linspace(-233, -207, 15)  # Fewer points for smaller figure
        y = np.linspace(-575, -555, 15)
        X, Y = np.meshgrid(x, y)
        
        k = 2 * np.pi / (self.wave_period * 20)
        propagation_angle = np.radians(self.wave_direction - 180 - 90)
        Z = self.wave_height * np.sin(k * (X * np.cos(propagation_angle) + 
                                           Y * np.sin(propagation_angle)))
        
        levels = np.linspace(-self.wave_height, self.wave_height, 6)  # Fewer levels
        contour = ax.contourf(X, Y, Z, levels=levels, cmap='RdBu_r', alpha=0.2)
        
        # Sparse wind field (3x3 grid for compact view)
        x_wind = np.linspace(-230, -210, 5)
        y_wind = np.linspace(-572, -558, 5)
        X_wind, Y_wind = np.meshgrid(x_wind, y_wind)
        
        force_angle = np.radians(self.wind_direction - 180 - 90)
        arrow_scale = 0.15  # Smaller arrows for compact figure
        U = np.ones_like(X_wind) * np.cos(force_angle) * self.wind_speed / 10 * arrow_scale
        V = np.ones_like(Y_wind) * np.sin(force_angle) * self.wind_speed / 10 * arrow_scale
        
        ax.quiver(X_wind, Y_wind, U, V, 
                 color='darkorange', alpha=0.7, scale=1/25, scale_units='xy',
                 width=0.005, headwidth=5, headlength=6)
        
        # USV with forces (centered and scaled down)
        usv_x, usv_y = -220, -565
        usv_length = 7  # Smaller than original 10
        usv_width = 3.5  # Smaller than original 5
        
        usv = Rectangle((usv_x - usv_width/2, usv_y - usv_length/2), 
                       usv_width, usv_length,
                       fc='gray', ec='black', linewidth=1.2, alpha=0.8)
        ax.add_patch(usv)
        
        # Force vectors (scaled appropriately)
        wind_force_angle = np.radians(-45 - 90)
        wave_force_angle = np.radians(-60 - 90)
        
        wind_fx = 8 * np.cos(wind_force_angle)  # Smaller than original 12
        wind_fy = 8 * np.sin(wind_force_angle)
        wave_fx = 7 * np.cos(wave_force_angle)  # Smaller than original 10
        wave_fy = 7 * np.sin(wave_force_angle)
        
        # Wind force arrow
        ax.arrow(usv_x, usv_y, wind_fx, wind_fy,
                head_width=2.0, head_length=1.5, fc='darkorange', ec='darkorange',
                linewidth=1.5, alpha=0.9, label='Wind')
        
        # Wave force arrow
        ax.arrow(usv_x, usv_y, wave_fx, wave_fy,
                head_width=2.0, head_length=1.5, fc='blue', ec='darkblue',
                linewidth=1.5, alpha=0.9, label='Wave')
        
        # Resultant force
        total_fx = wind_fx + wave_fx
        total_fy = wind_fy + wave_fy
        ax.arrow(usv_x, usv_y, total_fx, total_fy,
                head_width=2.2, head_length=1.8, fc='purple', ec='purple',
                linewidth=1.8, alpha=0.9, linestyle='--',
                label='Resultant')
        
        # YAW MOMENT INDICATOR only
        moment_radius = 9
        
        # Draw partial circle arc to show rotation
        theta = np.linspace(-np.pi/2, np.pi/4, 30)
        arc_x = usv_x + moment_radius * np.cos(theta)
        arc_y = usv_y + moment_radius * np.sin(theta)
        ax.plot(arc_x, arc_y, 'purple', linewidth=1.8, linestyle=':', alpha=0.7)
        
        # Add curved arrow at the end of arc
        arc_end_angle = np.pi/4
        arc_end_x = usv_x + moment_radius * np.cos(arc_end_angle)
        arc_end_y = usv_y + moment_radius * np.sin(arc_end_angle)
        
        # Arrow head for the yaw moment
        arrow_angle = arc_end_angle + np.pi/2
        arrow_length = 2
        ax.arrow(arc_end_x, arc_end_y, 
                arrow_length * np.cos(arrow_angle), 
                arrow_length * np.sin(arrow_angle),
                head_width=1.5, head_length=1.2, 
                fc='purple', ec='purple', linewidth=1.5, alpha=0.8)
        
        # NO axis labels
        ax.set_xticks([])
        ax.set_yticks([])
        
        # NO legend
        
        # Grid
        ax.grid(True, alpha=0.15, linestyle=':', linewidth=0.4)
        
        # NO scale bar
        
        # Remove top and right spines for cleaner look
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        
        # Adjust layout
        plt.tight_layout()
        
        # Save figure
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Compact environmental forces figure saved to {save_path}")
        
        return fig

if __name__ == "__main__":
    generator = EnvironmentalForcesPlot()
    fig = generator.generate_figure('environmental_forces_compact.pdf')
    
    plt.show()