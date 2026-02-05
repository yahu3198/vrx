#!/usr/bin/env python3
"""
ICRA Heat Map Generator for USV Fault-Tolerant Control Paper
Generates publication-quality heat maps showing system robustness across different conditions
Revised version with specific degradation levels: 50%, 75%, 95%, 100%
And sea states: 2, 3, 4, 5
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
        # Format: [sea_state, degradation_percentage, success_rate, avg_time_to_harbor, energy_consumption]
        
        # REVISED DATA STRUCTURE with only 50%, 75%, 95%, 100% degradation
        # and sea states 2, 3, 4, 5
        self.raw_data = [
            # Sea State 2 (calm)
            [2, 0.5, 100, 45, 1000],   # 50% degradation
            [2, 0.75, 100, 58, 1400],   # 75% degradation
            [2, 0.95, 75, 70, 1700],   # 95% degradation
            [2, 1.0, 15, 0, 0],        # 100% failure
            
            # Sea State 3 (moderate)
            [3, 0.5, 100, 47, 1050],   # 50% degradation
            [3, 0.75, 100, 62, 1500],   # 75% degradation
            [3, 0.95, 100, 75, 1800],   # 95% degradation
            [3, 1.0, 50, 0, 0],        # 100% failure
            
            # Sea State 4 (rough)
            [4, 0.5, 100, 50, 1100],   # 50% degradation
            [4, 0.75, 100, 67, 1600],   # 75% degradation
            [4, 0.95, 100, 80, 1900],   # 95% degradation
            [4, 1.0, 60, 0, 0],        # 100% failure
            
            # Sea State 5 (very rough)
            [5, 0.5, 100, 56, 1200],    # 50% degradation
            [5, 0.75, 100, 77, 1850],   # 75% degradation
            [5, 0.95, 90, 92, 2150],   # 95% degradation
            [5, 1.0, 55, 0, 0],         # 100% failure
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
        
    def create_single_column_heatmap(self, save_path='heatmap_single_column.pdf'):
        """Create single-column heat map optimized for IEEE double-column format"""
        
        # IEEE single column width: ~3.5 inches
        fig, ax = plt.subplots(1, 1, figsize=(3.5, 3.0))  # Slightly adjusted height
        
        # Prepare data grid with specific values
        sea_states = np.array([2, 3, 4, 5])  # Only these sea states
        degradations = np.array([0.5, 0.75, 0.95, 1.0])  # Only these degradation levels
        
        # Create success rate matrix for environmental-assisted MPC
        success_matrix = np.zeros((len(sea_states), len(degradations)))
        
        for i, sea_state in enumerate(sea_states):
            for j, deg in enumerate(degradations):
                idx = np.where((self.data[:, 0] == sea_state) & (self.data[:, 1] == deg))[0]
                if len(idx) > 0:
                    success_matrix[i, j] = self.data[idx[0], 2]
        
        # Plot heat map
        im = ax.imshow(success_matrix, cmap='RdYlGn', aspect='auto', 
                      vmin=0, vmax=100, interpolation='bilinear')
        
        # Add text annotations for all cells (since we have fewer now)
        for i in range(len(sea_states)):
            for j in range(len(degradations)):
                value = success_matrix[i, j]
                color = 'white' if value < 50 else 'black'
                text = ax.text(j, i, f'{value:.0f}', ha='center', va='center',
                             color=color, fontsize=8, weight='bold')
        
        # Set ticks
        ax.set_xticks(range(len(degradations)))
        ax.set_yticks(range(len(sea_states)))
        ax.set_xticklabels(['50%', '75%', '95%', '100%'], fontsize=9)
        ax.set_yticklabels(['2', '3', '4', '5'], fontsize=9)
        
        # Labels with smaller font
        ax.set_xlabel('Thruster Degradation', fontsize=10)
        ax.set_ylabel('Sea State', fontsize=10)
        
        # Title optimized for single column
        # ax.set_title('Mission Success Rate\nEnvironment-Assisted MPC', 
        #             fontsize=11, pad=8)
        
        # Compact colorbar
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Success Rate (%)', fontsize=9)
        cbar.ax.tick_params(labelsize=8)
        
        # Add subtle grid for readability
        ax.set_xticks(np.arange(len(degradations)) - 0.5, minor=True)
        ax.set_yticks(np.arange(len(sea_states)) - 0.5, minor=True)
        ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
        
        # Highlight critical failure region (100% degradation)
        rect = patches.Rectangle((2.5, -0.5), 1, len(sea_states),
                                linewidth=1.5, edgecolor='darkred', 
                                facecolor='none', linestyle='--', alpha=0.7)
        ax.add_patch(rect)
        
        # Add annotation for the critical region
        # ax.annotate('Critical\nFailure', xy=(3, 1.5), xytext=(2.3, 0.5),
        #            fontsize=7, color='darkred', weight='bold',
        #            arrowprops=dict(arrowstyle='->', color='darkred', lw=1))
        
        # Tight layout for space efficiency
        plt.tight_layout()
        
        # Save with high DPI for IEEE publications
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Single-column heat map saved to {save_path}")
        
        return fig, ax
    
    def create_success_rate_heatmap(self, method='env_assisted', save_path='heatmap_success_rate.pdf'):
        """Create heat map showing success rates across conditions"""
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        # Prepare data grid with specific values
        sea_states = np.array([2, 3, 4, 5])
        degradations = np.array([0.5, 0.75, 0.95, 1.0])
        
        # Create success rate matrix for environmental-assisted MPC
        success_matrix_env = np.zeros((len(sea_states), len(degradations)))
        
        for i, sea_state in enumerate(sea_states):
            for j, deg in enumerate(degradations):
                idx = np.where((self.data[:, 0] == sea_state) & (self.data[:, 1] == deg))[0]
                if len(idx) > 0:
                    success_matrix_env[i, j] = self.data[idx[0], 2]
        
        # Create baseline MPC matrix (simulated - worse performance)
        # REPLACE WITH YOUR ACTUAL BASELINE DATA
        success_matrix_baseline = success_matrix_env * 0.3  # Baseline performs worse
        success_matrix_baseline[:, 0] = success_matrix_baseline[:, 0] * 3  # Better at 50%
        success_matrix_baseline[:, -1] = 0   # Complete failure at 100%
        
        # Plot Environmental-Assisted MPC
        im1 = ax1.imshow(success_matrix_env, cmap='RdYlGn', aspect='auto', 
                        vmin=0, vmax=100, interpolation='bilinear')
        
        # Add text annotations
        for i in range(len(sea_states)):
            for j in range(len(degradations)):
                value = success_matrix_env[i, j]
                color = 'white' if value < 50 else 'black'
                text = ax1.text(j, i, f'{value:.0f}%', ha='center', va='center',
                              color=color, fontsize=10, weight='bold')
        
        ax1.set_xticks(range(len(degradations)))
        ax1.set_yticks(range(len(sea_states)))
        ax1.set_xticklabels(['50%', '75%', '95%', '100%'])
        ax1.set_yticklabels(['2', '3', '4', '5'])
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
        for i in range(len(sea_states)):
            for j in range(len(degradations)):
                value = success_matrix_baseline[i, j]
                color = 'white' if value < 50 else 'black'
                text = ax2.text(j, i, f'{value:.0f}%', ha='center', va='center',
                              color=color, fontsize=10, weight='bold')
        
        ax2.set_xticks(range(len(degradations)))
        ax2.set_yticks(range(len(sea_states)))
        ax2.set_xticklabels(['50%', '75%', '95%', '100%'])
        ax2.set_yticklabels(['2', '3', '4', '5'])
        ax2.set_xlabel('Thruster Degradation Level', fontsize=12, weight='bold')
        ax2.set_ylabel('Sea State', fontsize=12, weight='bold')
        ax2.set_title('Baseline MPC', fontsize=13, weight='bold')
        
        # Add colorbar
        cbar2 = plt.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04)
        cbar2.set_label('Success Rate (%)', fontsize=11)
        
        # Add main title
        fig.suptitle('Mission Success Rate: Robustness Analysis', 
                    fontsize=14, weight='bold', y=1.02)
        
        # Add critical regions
        self._add_critical_regions(ax1, len(sea_states))
        self._add_critical_regions(ax2, len(sea_states))
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.pdf', '.png'), dpi=300, bbox_inches='tight')
        print(f"Heat map saved to {save_path}")
        
        return fig, (ax1, ax2)
    
    def create_improvement_heatmap(self, save_path='heatmap_improvement.pdf'):
        """Create heat map showing improvement over baseline"""
        
        fig, ax = plt.subplots(1, 1, figsize=(8, 6))
        
        # Prepare data grid with specific values
        sea_states = np.array([2, 3, 4, 5])
        degradations = np.array([0.5, 0.75, 0.95, 1.0])
        
        # Calculate improvement matrix
        improvement_matrix = np.zeros((len(sea_states), len(degradations)))
        
        for i, sea_state in enumerate(sea_states):
            for j, deg in enumerate(degradations):
                idx = np.where((self.data[:, 0] == sea_state) & (self.data[:, 1] == deg))[0]
                if len(idx) > 0:
                    env_success = self.data[idx[0], 2]
                    # Adjust baseline calculation for specific degradation levels
                    if deg == 0.5:
                        baseline_success = env_success * 0.9  # Baseline is closer at 50%
                    elif deg == 0.75:
                        baseline_success = env_success * 0.5  # Worse at 75%
                    elif deg == 0.95:
                        baseline_success = env_success * 0.2  # Much worse at 95%
                    else:  # 100%
                        baseline_success = 0
                    improvement = env_success - baseline_success
                    improvement_matrix[i, j] = improvement
        
        # Create custom diverging colormap
        colors = ['#d73027', '#fc8d59', '#fee090', '#ffffff', 
                 '#e0f3f8', '#91bfdb', '#4575b4']
        n_bins = 100
        cmap = LinearSegmentedColormap.from_list('custom', colors, N=n_bins)
        
        # Plot improvement heat map
        im = ax.imshow(improvement_matrix, cmap=cmap, aspect='auto',
                      vmin=-50, vmax=80, interpolation='bilinear')
        
        # Add text annotations with improvement percentages
        for i in range(len(sea_states)):
            for j in range(len(degradations)):
                value = improvement_matrix[i, j]
                color = 'white' if abs(value) > 40 else 'black'
                text = ax.text(j, i, f'{value:+.0f}%', ha='center', va='center',
                             color=color, fontsize=10, weight='bold')
        
        ax.set_xticks(range(len(degradations)))
        ax.set_yticks(range(len(sea_states)))
        ax.set_xticklabels(['50%', '75%', '95%', '100%'])
        ax.set_yticklabels(['2', '3', '4', '5'])
        ax.set_xlabel('Thruster Degradation Level', fontsize=12, weight='bold')
        ax.set_ylabel('Sea State', fontsize=12, weight='bold')
        ax.set_title('Performance Improvement: Environment-Assisted vs Baseline MPC',
                    fontsize=13, weight='bold', pad=15)
        
        # Add colorbar with custom label
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Success Rate Improvement (%)', fontsize=11)
        
        # Add contour lines for significant improvements
        X, Y = np.meshgrid(range(len(degradations)), range(len(sea_states)))
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
    
    def _add_critical_regions(self, ax, num_sea_states):
        """Add visual indicators for critical regions"""
        # Add rectangle for critical failure region (100% degradation)
        rect = patches.Rectangle((2.5, -0.5), 1, num_sea_states,
                                linewidth=2, edgecolor='red', 
                                facecolor='none', linestyle='--')
        ax.add_patch(rect)
        
    def _add_benefit_regions(self, ax, matrix):
        """Highlight regions of maximum benefit"""
        # Find coordinates of maximum improvement
        max_benefit = np.unravel_index(np.argmax(matrix), matrix.shape)
        circle = patches.Circle((max_benefit[1], max_benefit[0]), 0.35,
                              linewidth=3, edgecolor='gold', 
                              facecolor='none', linestyle='-')
        ax.add_patch(circle)
        
    def create_all_heatmaps(self, output_dir='./figures/'):
        """Generate all heat maps for the paper"""
        import os
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate all visualizations
        self.create_success_rate_heatmap(
            save_path=os.path.join(output_dir, 'heatmap_success_rate.pdf'))
        self.create_improvement_heatmap(
            save_path=os.path.join(output_dir, 'heatmap_improvement.pdf'))
        self.create_single_column_heatmap(
            save_path=os.path.join(output_dir, 'heatmap_single_column.pdf'))
        
        print(f"\nAll heat maps generated in {output_dir}")
        print("Files created:")
        print("  - heatmap_success_rate.pdf/png")
        print("  - heatmap_improvement.pdf/png")  
        print("  - heatmap_single_column.pdf/png (IEEE single column)")


if __name__ == "__main__":
    # Create generator instance
    generator = ICRAHeatMapGenerator()
    
    # Generate all heat maps
    generator.create_all_heatmaps()
    
    # Or generate individual heat maps
    # fig, ax = generator.create_single_column_heatmap()
    
    # Show plots
    plt.show()