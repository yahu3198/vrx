#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon
from matplotlib.collections import LineCollection
import matplotlib.patheffects as path_effects
from matplotlib import cm
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

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
        
        # Increase base font size
        plt.rcParams['font.size'] = 12
        plt.rcParams['axes.labelsize'] = 13
        plt.rcParams['xtick.labelsize'] = 12
        plt.rcParams['ytick.labelsize'] = 12
        
        fig, ax = plt.subplots(figsize=(5, 7), dpi=150)
        ax.set_aspect('equal')
        
        # TIGHTER plot limits to focus on important area
        ax.set_xlim(-250, -190)
        ax.set_ylim(-615, -515)
        
        # 1. Create wave field background (rotated)
        self._draw_wave_field_rotated(ax)
        
        # 2. Overlay wind vectors (rotated and shorter)
        self._draw_wind_field_rotated(ax)
        
        # 3. Add USV with force visualization (rotated)
        # Position adjusted for tighter limits
        self._draw_usv_with_forces_rotated(ax, -215, -560)
        
        # 4. Add legend and annotations
        self._add_annotations(ax)
        
        # Title with larger font
        ax.set_title('Environmental Forces as Virtual Actuators', 
                    fontsize=15, weight='bold', pad=15)
        
        # Labels with larger font
        ax.set_xlabel('North (m)', fontsize=13, weight='bold')
        ax.set_ylabel('East (m)', fontsize=13, weight='bold')
        
        # Grid
        ax.grid(True, alpha=0.2, linestyle=':', linewidth=0.5)
        ax.tick_params(labelsize=11)
        
        # Remove top and right spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        
        # Use subplots_adjust instead of tight_layout to avoid the error
        plt.subplots_adjust(left=0.1, right=0.95, top=0.95, bottom=0.1)
        
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
        """Draw wave height field as background (rotated) with INSET colorbar"""
        # Create grid in rotated space
        x = np.linspace(-260, -180, 40)
        y = np.linspace(-615, -510, 40)
        X, Y = np.meshgrid(x, y)
        
        # Wave parameters - adjust angle for rotation
        k = 2 * np.pi / (self.wave_period * 20)
        propagation_angle = np.radians(self.wave_direction - 180 - 90)
        
        Z = self.wave_height * np.sin(k * (X * np.cos(propagation_angle) + 
                                           Y * np.sin(propagation_angle)))
        
        # Plot as contour with more transparency
        levels = np.linspace(-self.wave_height, self.wave_height, 12)
        contour = ax.contourf(X, Y, Z, levels=levels, cmap='RdBu_r', alpha=0.25)
        
        # Add wave crests with less prominence
        ax.contour(X, Y, Z, levels=[0.5, 1.0], colors='blue', 
                  alpha=0.15, linewidths=0.8)
        ax.contour(X, Y, Z, levels=[-1.0, -0.5], colors='red', 
                  alpha=0.15, linewidths=0.8)
        
        # Create inset axes for colorbar
        axins = inset_axes(ax, 
                          width="40%",
                          height="3%", 
                          loc='upper right',
                          borderpad=2.5)
        
        cbar = plt.colorbar(contour, cax=axins, orientation='horizontal')
        cbar.set_label('Wave Height (m)', fontsize=12)
        cbar.ax.tick_params(labelsize=11)
        cbar.set_ticks([-1.5, 0, 1.5])
        cbar.ax.xaxis.set_ticks_position('top')
        cbar.ax.xaxis.set_label_position('top')
    
    def _draw_wind_field_rotated(self, ax):
        """Draw wind vector field (rotated and shorter arrows)"""
        # Create sparser grid for cleaner look
        x_wind = np.linspace(-250, -190, 6)
        y_wind = np.linspace(-610, -520, 6)
        X_wind, Y_wind = np.meshgrid(x_wind, y_wind)
        
        # Rotate wind direction for new coordinate system
        force_angle = np.radians(self.wind_direction - 180 - 90)
        
        # SHORTER arrows with better visibility
        arrow_scale = 0.25
        U = np.ones_like(X_wind) * np.cos(force_angle) * self.wind_speed / 10 * arrow_scale
        V = np.ones_like(Y_wind) * np.sin(force_angle) * self.wind_speed / 10 * arrow_scale
        
        # Add small variation for realism
        noise_scale = 0.015
        U += np.random.normal(0, noise_scale, U.shape)
        V += np.random.normal(0, noise_scale, V.shape)
        
        # Draw wind vectors with better visibility
        scale = 40
        ax.quiver(X_wind, Y_wind, U, V, 
                 color='darkorange', alpha=0.8, scale=1/scale, scale_units='xy',
                 width=0.003, headwidth=5, headlength=6, 
                 edgecolor='orange', linewidth=0.8,
                 label=f'Wind')
    
    def _draw_usv_with_forces_rotated(self, ax, usv_x, usv_y):
        """Draw USV with acting forces (rotated) - FIXED LEFT-POINTING FORCES"""
        # Draw LARGER USV body
        usv_length = 15
        usv_width = 8
        
        usv = Rectangle((usv_x - usv_width/2, usv_y - usv_length/2), 
                       usv_width, usv_length,
                       fc='gray', ec='black', linewidth=2.5, alpha=0.85,
                       label='USV')
        ax.add_patch(usv)
        
        # Draw coordinate frame with larger arrows
        # ax.arrow(usv_x, usv_y, 0, -12, head_width=2.5, head_length=2.5,
        #         fc='red', ec='darkred', alpha=0.6, linewidth=2)
        # ax.text(usv_x, usv_y - 15, 'x', fontsize=12, color='red', weight='bold')
        
        # ax.arrow(usv_x, usv_y, 12, 0, head_width=2.5, head_length=2.5,
        #         fc='green', ec='darkgreen', alpha=0.6, linewidth=2)
        # ax.text(usv_x + 15, usv_y, 'y', fontsize=12, color='green', weight='bold')
        
        # FIXED: Forces push USV, so they point in OPPOSITE direction of where they come FROM
        # Wind FROM 135° means force points TO -45° (or 315°)
        # After rotation of -90°: -45° - 90° = -135°
        wind_force_angle = np.radians(-45 - 90)
        
        # Waves FROM 120° means force points TO -60° (or 300°)  
        # After rotation of -90°: -60° - 90° = -150°
        wave_force_angle = np.radians(-60 - 90)
        
        # LARGER force arrows for better visibility
        force_scale = 1.2
        wind_fx = 18 * np.cos(wind_force_angle) * force_scale
        wind_fy = 18 * np.sin(wind_force_angle) * force_scale
        
        wave_fx = 15 * np.cos(wave_force_angle) * force_scale
        wave_fy = 15 * np.sin(wave_force_angle) * force_scale
        
        # Draw force vectors with thicker lines
        ax.arrow(usv_x, usv_y, wind_fx, wind_fy,
                head_width=3.5, head_length=3, fc='darkorange', ec='darkorange',
                linewidth=3, alpha=0.9, label='Wind Force')
        
        ax.arrow(usv_x, usv_y, wave_fx, wave_fy,
                head_width=3.5, head_length=3, fc='blue', ec='darkblue',
                linewidth=3, alpha=0.9, label='Wave Force')
        
        # Draw resultant with more prominence
        total_fx = wind_fx + wave_fx
        total_fy = wind_fy + wave_fy
        ax.arrow(usv_x, usv_y, total_fx, total_fy,
                head_width=4, head_length=3.5, fc='purple', ec='purple',
                linewidth=3.5, alpha=0.95, linestyle='--',
                label='Resultant Force')
        
        # Add LARGER yaw moment indicator
        moment_radius = 18
        moment_arc = Circle((usv_x, usv_y), moment_radius, 
                           fill=False, ec='purple', 
                           linewidth=2.5, linestyle=':', alpha=0.7)
        ax.add_patch(moment_arc)
        
        # Curved arrow for moment with better visibility
        arc_angle = -45
        arc_x = usv_x + moment_radius * np.cos(np.radians(arc_angle))
        arc_y = usv_y + moment_radius * np.sin(np.radians(arc_angle))
        ax.annotate('', xy=(arc_x, arc_y),
                   xytext=(usv_x, usv_y - moment_radius),
                   arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.3',
                                 color='purple', linewidth=2.5))
        ax.text(usv_x + 8, usv_y - moment_radius - 5, 'M_z', 
               fontsize=13, color='purple', weight='bold')
    
    def _add_annotations(self, ax):
        """Add key annotations and legend with LARGER text"""
        # Legend with larger font
        ax.legend(loc='lower right', fontsize=11, framealpha=0.95,
                 edgecolor='black', frameon=True)
        
        # Scale reference with better positioning
        ax.plot([-250, -230], [-612, -612], 'k-', linewidth=3)
        ax.text(-240, -610, '20 m', ha='center', fontsize=11, weight='bold')

if __name__ == "__main__":
    generator = PanelCGenerator()
    fig, ax = generator.generate_panel_c('icra_figure1_panel_c_fixed.pdf')
    plt.show()