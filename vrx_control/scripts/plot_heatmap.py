#!/usr/bin/env python3
"""
ICRA Heat Map Generator for USV Fault-Tolerant Control Paper
Generates publication-quality heat maps showing system robustness across different conditions
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import LinearSegmentedColormap
import seaborn as sns
from scipy.interpolate import interp2d
from matplotlib import cm
from matplotlib.colors import ListedColormap
import pandas as pd

# Set publication quality defaults
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans']
plt.rcParams['font.size'] = 11
plt.rcParams['axes.linewidth'] = 1.5
plt.rcParams['xtick.major.width'] = 1.5
plt.rcParams['ytick.major.width'] = 1.5
plt.rcParams['xtick.major.size'] = 5
plt.rcParams['ytick.major.size'] = 5

class ICRAHeatMapGenerator:
    def __init__(self):
        """Initialize the heat map generator with performance data"""
        # Define your experimental data here
        # Format: [wind_speed, degradation_percentage, success_rate, avg_time_to_harbor, energy_consumption]
        
        # Sample data structure - REPLACE WITH YOUR ACTUAL DATA
        self.raw_data = [
            # Wind 0 m/s
            [2, 0.5, 100, 45, 1000],   # No degradation
            [2, 0.6, 100, 48, 1100],  # 25% degradation
            [2, 0.7, 100, 52, 1250],   # 50% degradation
            [2, 0.8, 95, 58, 1400],   # 75% degradation
            [2, 0.9, 85, 65, 1600],    # 90% degradation
            [2, 0.95, 80, 70, 1700],   # 95% degradation
            [2, 1.0, 15, 0, 0],        # Complete failure
            
            # Wind 5 m/s
            [3, 0.5, 100, 47, 1050],
            [3, 0.6, 100, 50, 1150],
            [3, 0.7, 98, 55, 1300],
            [3, 0.8, 92, 62, 1500],
            [3, 0.9, 82, 68, 1650],
            [3, 0.95, 75, 75, 1800],
            [3, 1.0, 12, 0, 0],
            
            # Wind 10 m/s
            [4, 0.5, 100, 50, 1100],
            [4, 0.6, 98, 53, 1200],
            [4, 0.7, 95, 58, 1400],
            [4, 0.8, 88, 67, 1600],
            [4, 0.9, 78, 73, 1750],
            [4, 0.95, 70, 80, 1900],
            [4, 1.0, 10, 0, 0],
            
            # Wind 15 m/s
            [4.5, 0.5, 98, 53, 1150],
            [4.5, 0.6, 95, 57, 1300],
            [4.5, 0.7, 90, 63, 1500],
            [4.5, 0.8, 82, 72, 1700],
            [4.5, 0.9, 72, 78, 1850],
            [4.5, 0.95, 65, 85, 2000],
            [4.5, 1.0, 8, 0, 0],
            
            # Wind 20 m/s
            [5, 0.5, 95, 56, 1200],
            [5, 0.6, 92, 60, 1350],
            [5, 0.7, 85, 67, 1600],
            [5, 0.8, 75, 77, 1850],
            [5, 0.9, 65, 85, 2000],
            [5, 0.95, 58, 92, 2150],
            [5, 1.0, 5, 0, 0],
        ]
        
        # Convert to numpy array for easier manipulation
        self.data = np.array(self.raw_data)
        
        # Define color schemes for different metrics
        self.colormaps = {
            'success_rate': 'RdYlGn',  # Red-Yellow-Green for success
            'time': 'YlOrRd_r',        # Reversed for time (lower is better)
            'energy': 'YlOrRd_r',       # Reversed for energy (lower is better)
            'comparison': 'RdBu'        # Diverging for comparison
        }
        
    def create_success_rate_heatmap(self, method='env_assisted', save_path='heatmap_success_rate.pdf'):
        """Create heat map showing success rates across conditions"""
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        # Prepare data grid
        wind_speeds = np.unique(self.data[:, 0])
        degradations = np.unique(self.data[:, 1])
        
        # Create success rate matrix for environmental-assisted MPC
        success_matrix_env = np.zeros((len(wind_speeds), len(degradations)))
        
        for i, wind in enumerate(wind_speeds):
            for j, deg in enumerate(degradations):
                idx = np.where((self.data[:, 0] == wind) & (self.data[:, 1] == deg))[0]
                if len(idx) > 0:
                    success_matrix_env[i, j] = self.data[idx[0], 2]
        
        # Create baseline MPC matrix (simulated - worse performance)
        # REPLACE WITH YOUR ACTUAL BASELINE DATA
        success_matrix_baseline = success_matrix_env * 0.3  # Baseline performs worse
        success_matrix_baseline[:, 0] = 100  # No degradation = full success
        success_matrix_baseline[:, -1] = 0   # Complete failure = no success
        
        # Plot Environmental-Assisted MPC
        im1 = ax1.imshow(success_matrix_env, cmap='RdYlGn', aspect='auto', 
                        vmin=0, vmax=100, interpolation='bilinear')
        
        # Add text annotations
        for i in range(len(wind_speeds)):
            for j in range(len(degradations)):
                value = success_matrix_env[i, j]
                color = 'white' if value < 50 else 'black'
                text = ax1.text(j, i, f'{value:.0f}%', ha='center', va='center',
                              color=color, fontsize=9, weight='bold')
        
        ax1.set_xticks(range(len(degradations)))
        ax1.set_yticks(range(len(wind_speeds)))
        ax1.set_xticklabels([f'{int(d*100)}%' for d in degradations])
        ax1.set_yticklabels([f'{int(w)}' for w in wind_speeds])
        ax1.set_xlabel('Thruster Degradation Level', fontsize=12, weight='bold')
        ax1.set_ylabel('Sea State', fontsize=12, weight='bold')
        ax1.set_title('Environment-Assisted MPC', fontsize=13, weight='bold')
        
        # Add colorbar
        cbar1 = plt.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04)
        cbar1.set_label('Success Rate (%)', fontsize=11)
        
        # Plot Baseline MPC
        im2 = ax2.imshow(success_matrix_baseline, cmap='RdYlGn', aspect='auto',
                        vmin=0, vmax=100, interpolation='bilinear')
        
        # Add text annotations
        for i in range(len(wind_speeds)):
            for j in range(len(degradations)):
                value = success_matrix_baseline[i, j]
                color = 'white' if value < 50 else 'black'
                text = ax2.text(j, i, f'{value:.0f}%', ha='center', va='center',
                              color=color, fontsize=9, weight='bold')
        
        ax2.set_xticks(range(len(degradations)))
        ax2.set_yticks(range(len(wind_speeds)))
        ax2.set_xticklabels([f'{int(d*100)}%' for d in degradations])
        ax2.set_yticklabels([f'{int(w)}' for w in wind_speeds])
        ax2.set_xlabel('Thruster Degradation Level', fontsize=12, weight='bold')
        ax2.set_ylabel('Sea States', fontsize=12, weight='bold')
        ax2.set_title('Baseline MPC', fontsize=13, weight='bold')
        
        # Add colorbar
        cbar2 = plt.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04)
        cbar2.set_label('Success Rate (%)', fontsize=11)
        
        # Add main title
        fig.suptitle('Mission Success Rate: Robustness Analysis', 
                    fontsize=14, weight='bold', y=1.02)
        
        # Add critical regions
        self._add_critical_regions(ax1)
        self._add_critical_regions(ax2)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Heat map saved to {save_path}")
        
        return fig, (ax1, ax2)
    
    def create_improvement_heatmap(self, save_path='heatmap_improvement.pdf'):
        """Create heat map showing improvement over baseline"""
        
        fig, ax = plt.subplots(1, 1, figsize=(8, 6))
        
        # Prepare data grid
        wind_speeds = np.unique(self.data[:, 0])
        degradations = np.unique(self.data[:, 1])
        
        # Calculate improvement matrix
        improvement_matrix = np.zeros((len(wind_speeds), len(degradations)))
        
        for i, wind in enumerate(wind_speeds):
            for j, deg in enumerate(degradations):
                idx = np.where((self.data[:, 0] == wind) & (self.data[:, 1] == deg))[0]
                if len(idx) > 0:
                    env_success = self.data[idx[0], 2]
                    baseline_success = env_success * 0.3 if deg > 0 else 100
                    improvement = env_success - baseline_success
                    improvement_matrix[i, j] = improvement
        
        # Create custom diverging colormap
        colors = ['#d73027', '#fc8d59', '#fee090', '#ffffff', 
                 '#e0f3f8', '#91bfdb', '#4575b4']
        n_bins = 100
        cmap = LinearSegmentedColormap.from_list('custom', colors, N=n_bins)
        
        # Plot improvement heat map
        im = ax.imshow(improvement_matrix, cmap=cmap, aspect='auto',
                      vmin=-50, vmax=50, interpolation='bilinear')
        
        # Add text annotations with improvement percentages
        for i in range(len(wind_speeds)):
            for j in range(len(degradations)):
                value = improvement_matrix[i, j]
                color = 'white' if abs(value) > 25 else 'black'
                text = ax.text(j, i, f'{value:+.0f}%', ha='center', va='center',
                             color=color, fontsize=9, weight='bold')
        
        ax.set_xticks(range(len(degradations)))
        ax.set_yticks(range(len(wind_speeds)))
        ax.set_xticklabels([f'{int(d*100)}%' for d in degradations])
        ax.set_yticklabels([f'{int(w)}' for w in wind_speeds])
        ax.set_xlabel('Thruster Degradation Level', fontsize=12, weight='bold')
        ax.set_ylabel('Sea States', fontsize=12, weight='bold')
        ax.set_title('Performance Improvement: Environment-Assisted vs Baseline MPC',
                    fontsize=13, weight='bold', pad=15)
        
        # Add colorbar with custom label
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Success Rate Improvement (%)', fontsize=11)
        
        # Add contour lines for significant improvements
        X, Y = np.meshgrid(range(len(degradations)), range(len(wind_speeds)))
        contours = ax.contour(X, Y, improvement_matrix, levels=[20, 40, 60],
                            colors='black', linewidths=1.5, alpha=0.4)
        ax.clabel(contours, inline=True, fontsize=8)
        
        # Highlight regions of maximum benefit
        self._add_benefit_regions(ax, improvement_matrix)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Improvement heat map saved to {save_path}")
        
        return fig, ax
    
    def create_operational_envelope(self, save_path='operational_envelope.pdf'):
        """Create operational envelope visualization"""
        
        fig, ax = plt.subplots(1, 1, figsize=(9, 6))
        
        # Prepare data
        wind_speeds = np.unique(self.data[:, 0])
        degradations = np.unique(self.data[:, 1]) * 100  # Convert to percentage
        
        # Define operational zones based on success rate
        success_threshold_high = 80  # High confidence operation
        success_threshold_med = 50   # Degraded but operational
        success_threshold_low = 20   # Marginal operation
        
        # Create success rate matrix
        success_matrix = np.zeros((len(wind_speeds), len(degradations)))
        for i, wind in enumerate(wind_speeds):
            for j, deg_pct in enumerate(degradations):
                idx = np.where((self.data[:, 0] == wind) & 
                             (self.data[:, 1] == deg_pct/100))[0]
                if len(idx) > 0:
                    success_matrix[i, j] = self.data[idx[0], 2]
        
        # Create operational zones
        zones_matrix = np.zeros_like(success_matrix)
        zones_matrix[success_matrix >= success_threshold_high] = 3  # Green zone
        zones_matrix[(success_matrix >= success_threshold_med) & 
                    (success_matrix < success_threshold_high)] = 2  # Yellow zone
        zones_matrix[(success_matrix >= success_threshold_low) & 
                    (success_matrix < success_threshold_med)] = 1  # Orange zone
        zones_matrix[success_matrix < success_threshold_low] = 0  # Red zone
        
        # Custom colormap for zones
        colors = ['#d73027', '#fc8d59', '#fee090', '#91cf60']
        n_bins = 4
        cmap = LinearSegmentedColormap.from_list('zones', colors, N=n_bins)
        
        # Plot zones
        im = ax.imshow(zones_matrix, cmap=cmap, aspect='auto', 
                      interpolation='nearest', alpha=0.8)
        
        # Add contour lines
        X, Y = np.meshgrid(degradations, wind_speeds)
        contours = ax.contour(X, Y, success_matrix, 
                            levels=[20, 50, 80],
                            colors='black', linewidths=2)
        ax.clabel(contours, inline=True, fontsize=10, fmt='%d%%')
        
        # Customize axes
        ax.set_xticks(np.arange(len(degradations)))
        ax.set_yticks(np.arange(len(wind_speeds)))
        ax.set_xticklabels([f'{int(d)}' for d in degradations])
        ax.set_yticklabels([f'{int(w)}' for w in wind_speeds])
        ax.set_xlabel('Thruster Degradation (%)', fontsize=12, weight='bold')
        ax.set_ylabel('Sea States', fontsize=12, weight='bold')
        ax.set_title('Operational Envelope with Environmental Assistance',
                    fontsize=13, weight='bold', pad=15)
        
        # Add legend
        legend_elements = [
            patches.Patch(color='#91cf60', label='Nominal Operation (>80%)'),
            patches.Patch(color='#fee090', label='Degraded Operation (50-80%)'),
            patches.Patch(color='#fc8d59', label='Marginal Operation (20-50%)'),
            patches.Patch(color='#d73027', label='Mission Failure (<20%)')
        ]
        ax.legend(handles=legend_elements, loc='upper right', 
                 fontsize=10, framealpha=0.9)
        
        # Add annotations for key regions
        ax.annotate('Safe Operating Region', xy=(25, 5), xytext=(30, 8),
                   arrowprops=dict(arrowstyle='->', color='green', lw=2),
                   fontsize=11, weight='bold', color='green')
        
        ax.annotate('Critical Fault Region', xy=(90, 15), xytext=(70, 18),
                   arrowprops=dict(arrowstyle='->', color='red', lw=2),
                   fontsize=11, weight='bold', color='red')
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Operational envelope saved to {save_path}")
        
        return fig, ax
    
    def _add_critical_regions(self, ax):
        """Add visual indicators for critical regions"""
        # Add rectangle for critical failure region (high degradation)
        # Position at the last column (100% degradation)
        rect = patches.Rectangle((5.5, -0.5), 1, len(np.unique(self.data[:, 0])),
                                linewidth=2, edgecolor='red', 
                                facecolor='none', linestyle='--')
        ax.add_patch(rect)
        
    def _add_benefit_regions(self, ax, matrix):
        """Highlight regions of maximum benefit"""
        # Find coordinates of maximum improvement
        max_benefit = np.unravel_index(np.argmax(matrix), matrix.shape)
        circle = patches.Circle((max_benefit[1], max_benefit[0]), 0.5,
                              linewidth=3, edgecolor='gold', 
                              facecolor='none', linestyle='-')
        ax.add_patch(circle)
        
    def create_single_column_heatmap(self, save_path='heatmap_single_column.pdf'):
        """Create single-column heat map optimized for IEEE double-column format"""
        
        # IEEE single column width: ~3.5 inches
        fig, ax = plt.subplots(1, 1, figsize=(3.5, 3.2))
        
        # Prepare data grid
        wind_speeds = np.unique(self.data[:, 0])
        degradations = np.unique(self.data[:, 1])
        
        # Create success rate matrix for environmental-assisted MPC
        success_matrix = np.zeros((len(wind_speeds), len(degradations)))
        
        for i, wind in enumerate(wind_speeds):
            for j, deg in enumerate(degradations):
                idx = np.where((self.data[:, 0] == wind) & (self.data[:, 1] == deg))[0]
                if len(idx) > 0:
                    success_matrix[i, j] = self.data[idx[0], 2]
        
        # Create custom colormap for better visual impact
        colors = ['#d73027', '#f46d43', '#fdae61', '#fee090', 
                 '#e0f3f8', '#abd9e9', '#74add1', '#4575b4']
        n_bins = 100
        cmap = LinearSegmentedColormap.from_list('custom_rdylgn', colors[::-1], N=n_bins)
        
        # Plot heat map
        im = ax.imshow(success_matrix, cmap='RdYlGn', aspect='auto', 
                      vmin=0, vmax=100, interpolation='bilinear')
        
        # Add text annotations - smaller font for single column
        for i in range(len(wind_speeds)):
            for j in range(len(degradations)):
                value = success_matrix[i, j]
                # Only show text for key cells to avoid clutter
                if j % 2 == 0 or j == len(degradations) - 1:  # Show every other column + last
                    color = 'white' if value < 50 else 'black'
                    text = ax.text(j, i, f'{value:.0f}', ha='center', va='center',
                                 color=color, fontsize=7, weight='bold')
        
        # Set ticks
        ax.set_xticks(range(len(degradations)))
        ax.set_yticks(range(len(wind_speeds)))
        ax.set_xticklabels([f'{int(d*100)}%' for d in degradations], fontsize=8)
        ax.set_yticklabels([f'{int(w)}' for w in wind_speeds], fontsize=8)
        
        # Labels with smaller font
        ax.set_xlabel('Thruster Degradation', fontsize=9, weight='bold')
        ax.set_ylabel('Sea States', fontsize=9, weight='bold')
        
        # Title optimized for single column
        ax.set_title('Mission Success Rate\nEnvironment-Assisted MPC', 
                    fontsize=10, weight='bold', pad=8)
        
        # Compact colorbar
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Success Rate (%)', fontsize=8)
        cbar.ax.tick_params(labelsize=7)
        
        # Add critical improvement annotation
        # Find region of maximum benefit (high degradation, moderate wind)
        max_benefit_region = success_matrix[2:4, 4:6]  # 10-15 m/s wind, 75-95% degradation
        if max_benefit_region.size > 0:
            avg_success = np.mean(max_benefit_region)
            # Add text box highlighting key advantage
            # props = dict(boxstyle='round,pad=0.3', facecolor='yellow', 
            #             alpha=0.7, edgecolor='orange', linewidth=1.5)
            # ax.text(0.98, 0.5, f'70% higher\nsuccess than\nbaseline in\ncritical region', 
            #        transform=ax.transAxes, fontsize=7, va='center', ha='right',
            #        bbox=props, weight='bold')
        
        # Add subtle grid for readability
        ax.set_xticks(np.arange(len(degradations)) - 0.5, minor=True)
        ax.set_yticks(np.arange(len(wind_speeds)) - 0.5, minor=True)
        ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
        
        # Highlight critical failure region with subtle box
        rect = patches.Rectangle((5.5, -0.5), 1, len(wind_speeds),
                                linewidth=1.5, edgecolor='darkred', 
                                facecolor='none', linestyle='--', alpha=0.7)
        ax.add_patch(rect)
        
        # Add small note about baseline performance
        # ax.text(0.5, -0.18, 'Baseline MPC: <20% success above 75% degradation',
        #        transform=ax.transAxes, ha='center', fontsize=7, 
        #        style='italic', color='gray')
        
        # Tight layout for space efficiency
        plt.tight_layout()
        
        # Save with high DPI for IEEE publications
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Single-column heat map saved to {save_path}")
        
        return fig, ax
    
    def create_all_heatmaps(self, output_dir='./figures/'):
        """Generate all heat maps for the paper"""
        import os
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate all visualizations
        self.create_success_rate_heatmap(
            save_path=os.path.join(output_dir, 'heatmap_success_rate.pdf'))
        self.create_improvement_heatmap(
            save_path=os.path.join(output_dir, 'heatmap_improvement.pdf'))
        self.create_operational_envelope(
            save_path=os.path.join(output_dir, 'operational_envelope.pdf'))
        self.create_single_column_heatmap(
            save_path=os.path.join(output_dir, 'heatmap_single_column.pdf'))
        
        print(f"\nAll heat maps generated in {output_dir}")
        print("Files created:")
        print("  - heatmap_success_rate.pdf/png")
        print("  - heatmap_improvement.pdf/png")  
        print("  - operational_envelope.pdf/png")
        print("  - heatmap_single_column.pdf/png (IEEE single column)")


if __name__ == "__main__":
    # Create generator instance
    generator = ICRAHeatMapGenerator()
    
    # Generate all heat maps
    generator.create_all_heatmaps()
    
    # Or generate individual heat maps
    # fig1, axes1 = generator.create_success_rate_heatmap()
    # fig2, ax2 = generator.create_improvement_heatmap()
    # fig3, ax3 = generator.create_operational_envelope()
    
    # Show plots
    plt.show()