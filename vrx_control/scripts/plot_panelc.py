#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon
from matplotlib.collections import LineCollection
import matplotlib.patheffects as path_effects
from matplotlib import cm

class PanelCGenerator:
    def __init__(self):
        # Environmental parameters from sydney_regatta.sdf
        self.wind_direction = 135  # degrees (FROM this direction)
        self.wind_speed = 6.5  # m/s
        self.wave_direction = 120  # degrees  
        self.wave_height = 1.5  # meters
        self.wave_period = 7.0  # seconds
        
    def generate_panel_c(self, save_path='figure1_panel_c.pdf'):
        """Generate Panel C showing environmental forces"""
        
        fig, ax = plt.subplots(figsize=(8, 8), dpi=150)
        ax.set_aspect('equal')
        
        # ROTATED plot limits (90 degrees anticlockwise to match panel B)
        ax.set_xlim(-270, -170)  # Swapped and negated from original y-limits
        ax.set_ylim(-620, -520)  # Negated from original x-limits
        
        # 1. Create wave field background (rotated)
        self._draw_wave_field_rotated(ax)
        
        # 2. Overlay wind vectors (rotated and shorter)
        self._draw_wind_field_rotated(ax)
        
        # 3. Add USV with force visualization (rotated)
        # Position rotated: original (-540, 220) -> rotated (-220, 540)
        self._draw_usv_with_forces_rotated(ax, -220, -570)
        
        # 4. Add legend and annotations
        self._add_annotations(ax)
        
        # Title
        ax.set_title('Environmental Forces as Virtual Actuators', 
                    fontsize=14, weight='bold', pad=20)
        
        # Labels (swapped for rotation)
        ax.set_xlabel('North (m)', fontsize=12, weight='bold')
        ax.set_ylabel('East (m)', fontsize=12, weight='bold')
        
        # Grid
        ax.grid(True, alpha=0.2, linestyle=':', linewidth=0.5)
        ax.tick_params(labelsize=10)
        
        # Remove top and right spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Panel C saved to {save_path}")
        
        return fig, ax
    
    def _rotate_coords(self, x, y):
        """Rotate coordinates 90 degrees anticlockwise"""
        x_rot = -y
        y_rot = x
        return x_rot, y_rot
    
    def _draw_wave_field_rotated(self, ax):
        """Draw wave height field as background (rotated)"""
        # Create grid in rotated space
        x = np.linspace(-270, -170, 50)
        y = np.linspace(-620, -520, 50)
        X, Y = np.meshgrid(x, y)
        
        # Wave parameters - adjust angle for rotation
        k = 2 * np.pi / (self.wave_period * 20)
        # Rotate wave direction by 90 degrees
        propagation_angle = np.radians(self.wave_direction - 180 - 90)
        
        Z = self.wave_height * np.sin(k * (X * np.cos(propagation_angle) + 
                                           Y * np.sin(propagation_angle)))
        
        # Plot as contour
        levels = np.linspace(-self.wave_height, self.wave_height, 15)
        contour = ax.contourf(X, Y, Z, levels=levels, cmap='RdBu_r', alpha=0.3)
        
        # Add wave crests
        ax.contour(X, Y, Z, levels=[0.5, 1.0, 1.5], colors='blue', 
                  alpha=0.2, linewidths=1)
        ax.contour(X, Y, Z, levels=[-1.5, -1.0, -0.5], colors='red', 
                  alpha=0.2, linewidths=1)
        
        # Add colorbar
        cbar = plt.colorbar(contour, ax=ax, orientation='vertical', 
                           pad=0.02, shrink=0.3, anchor=(1.0, 0.8))
        cbar.set_label('Wave Height (m)', fontsize=9)
        cbar.ax.tick_params(labelsize=8)
    
    def _draw_wind_field_rotated(self, ax):
        """Draw wind vector field (rotated and shorter arrows)"""
        # Create sparse grid in rotated space
        x_wind = np.linspace(-260, -180, 6)
        y_wind = np.linspace(-610, -530, 6)
        X_wind, Y_wind = np.meshgrid(x_wind, y_wind)
        
        # Rotate wind direction for new coordinate system
        force_angle = np.radians(self.wind_direction - 180 - 90)
        
        # SHORTER arrows - reduced magnitude
        arrow_scale = 0.3  # Make arrows 30% of original length
        U = np.ones_like(X_wind) * np.cos(force_angle) * self.wind_speed / 10 * arrow_scale
        V = np.ones_like(Y_wind) * np.sin(force_angle) * self.wind_speed / 10 * arrow_scale
        
        # Add small variation for realism
        noise_scale = 0.02  # Reduced noise too
        U += np.random.normal(0, noise_scale, U.shape)
        V += np.random.normal(0, noise_scale, V.shape)
        
        # Draw wind vectors with adjusted scale
        scale = 50  # Adjusted scale parameter
        ax.quiver(X_wind, Y_wind, U, V, 
                 color='orange', alpha=0.7, scale=1/scale, scale_units='xy',
                 width=0.002, headwidth=4, headlength=5, 
                 edgecolor='darkorange', linewidth=0.5,
                 label=f'Wind: {self.wind_speed} m/s')
    
    def _draw_usv_with_forces_rotated(self, ax, usv_x, usv_y):
        """Draw USV with acting forces (rotated)"""
        # Draw USV body (rotated orientation)
        usv_length = 12
        usv_width = 6
        # For rotated view, swap dimensions
        usv = Rectangle((usv_x - usv_width/2, usv_y - usv_length/2), 
                       usv_width, usv_length,
                       fc='gray', ec='black', linewidth=2, alpha=0.8,
                       label='USV')
        ax.add_patch(usv)
        
        # Draw rotated coordinate frame
        # Original x-axis becomes -y-axis, original y-axis becomes x-axis
        ax.arrow(usv_x, usv_y, 0, -10, head_width=2, head_length=2,
                fc='red', ec='darkred', alpha=0.5)
        ax.text(usv_x, usv_y - 12, 'x', fontsize=10, color='red', weight='bold')
        
        ax.arrow(usv_x, usv_y, 10, 0, head_width=2, head_length=2,
                fc='green', ec='darkgreen', alpha=0.5)
        ax.text(usv_x + 12, usv_y, 'y', fontsize=10, color='green', weight='bold')
        
        # Calculate force magnitudes (scaled for visualization)
        # Rotate force directions by 90 degrees
        wind_force_angle = np.radians(self.wind_direction - 180 - 90)
        wave_force_angle = np.radians(self.wave_direction - 180 - 90)
        
        # SHORTER force arrows
        force_scale = 0.8  # Reduce force arrow length
        wind_fx = 15 * np.cos(wind_force_angle) * force_scale
        wind_fy = 15 * np.sin(wind_force_angle) * force_scale
        
        wave_fx = 12 * np.cos(wave_force_angle) * force_scale
        wave_fy = 12 * np.sin(wave_force_angle) * force_scale
        
        # Draw force vectors on USV
        ax.arrow(usv_x, usv_y, wind_fx, wind_fy,
                head_width=3, head_length=2, fc='orange', ec='darkorange',
                linewidth=2.5, alpha=0.8, label='Wind Force')
        
        ax.arrow(usv_x, usv_y, wave_fx, wave_fy,
                head_width=3, head_length=2, fc='blue', ec='darkblue',
                linewidth=2.5, alpha=0.8, label='Wave Force')
        
        # Draw resultant
        total_fx = wind_fx + wave_fx
        total_fy = wind_fy + wave_fy
        ax.arrow(usv_x, usv_y, total_fx, total_fy,
                head_width=3, head_length=2, fc='purple', ec='purple',
                linewidth=3, alpha=0.9, linestyle='--',
                label='Resultant Force')
        
        # Add yaw moment indicator
        moment_radius = 15
        moment_arc = Circle((usv_x, usv_y), moment_radius, 
                           fill=False, ec='purple', 
                           linewidth=2, linestyle=':', alpha=0.6)
        ax.add_patch(moment_arc)
        
        # Curved arrow for moment (adjusted for rotation)
        arc_angle = -45  # Adjusted for rotated view
        arc_x = usv_x + moment_radius * np.cos(np.radians(arc_angle))
        arc_y = usv_y + moment_radius * np.sin(np.radians(arc_angle))
        ax.annotate('', xy=(arc_x, arc_y),
                   xytext=(usv_x, usv_y - moment_radius),
                   arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.3',
                                 color='purple', linewidth=2))
        ax.text(usv_x + 5, usv_y - moment_radius - 5, 'M_z', 
               fontsize=11, color='purple', weight='bold')
    
    def _add_annotations(self, ax):
        """Add key annotations and legend"""
        # Virtual actuator explanation box
        explanation_text = (
            "Virtual Actuator Effect:\n"
            "• ~15° force misalignment → Yaw moment\n"
            "• Wind: average 6.5 m/s from ~135°\n"
            "• Waves: 1.5m height, 7s period from ~120°\n"
            "• Sea State 4 (Moderate conditions)"
        )
        
        props = dict(boxstyle='round,pad=0.5', facecolor='lightyellow', 
                    alpha=0.9, edgecolor='black', linewidth=1.5)
        ax.text(0.02, 0.98, explanation_text, transform=ax.transAxes,
               fontsize=10, va='top', bbox=props)
        
        # Add legend
        ax.legend(loc='lower right', fontsize=10, framealpha=0.9)
        
        # Add scale reference (rotated position)
        ax.plot([-265, -245], [-615, -615], 'k-', linewidth=3)
        ax.text(-255, -618, '20 m', ha='center', fontsize=9)

if __name__ == "__main__":
    generator = PanelCGenerator()
    fig, ax = generator.generate_panel_c('figure1_panel_c.pdf')
    plt.show()