import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def plot_harbor_zones():
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    
    # Define harbor zones (safe approach areas) - CORRECTED ORDER
    harbor_zones = [
        # Zone 1: [-580, 258], [-575, 240], [-600, 236], [-600, 248]
        [[-580, 258], [-575, 240], [-600, 236], [-600, 248]],
        # Zone 2: [-575, 222], [-575, 208], [-595, 208], [-595, 220] 
        [[-575, 222], [-575, 208], [-595, 208], [-595, 220]],
        # Zone 3: [-573, 192], [-593, 191], [-593, 183], [-584, 184]
        [[-573, 192], [-593, 191], [-593, 183], [-584, 184]]
    ]
    
    # Define dock areas (to be avoided) - CORRECTED ORDER
    dock_areas = [
        # Dock 1: between harbor zones 1 and 2
        [[-575, 240], [-575, 222], [-595, 220], [-600, 236]],
        # Dock 2: between harbor zones 2 and 3  
        [[-575, 208], [-573, 192], [-593, 191], [-595, 208]]
    ]
    
    # Plot harbor zones (green - safe areas)
    for i, zone in enumerate(harbor_zones):
        # Convert to numpy array for easier handling
        zone_array = np.array(zone)
        
        # Create polygon patch
        harbor_patch = patches.Polygon(zone_array, closed=True, 
                                     facecolor='lightgreen', edgecolor='green', 
                                     linewidth=2, alpha=0.7)
        ax.add_patch(harbor_patch)
        
        # Add zone labels
        center_x = np.mean(zone_array[:, 0])
        center_y = np.mean(zone_array[:, 1])
        ax.text(center_x, center_y, f'Harbor\nZone {i+1}', 
                ha='center', va='center', fontweight='bold', color='darkgreen')
        
        # Plot vertices with coordinates
        for j, (x, y) in enumerate(zone):
            ax.plot(x, y, 'go', markersize=6)
            ax.annotate(f'({x},{y})', (x, y), xytext=(5, 5), 
                       textcoords='offset points', fontsize=8, color='green')
    
    # Plot dock areas (red - avoid areas)
    for i, dock in enumerate(dock_areas):
        dock_array = np.array(dock)
        
        # Create polygon patch
        dock_patch = patches.Polygon(dock_array, closed=True,
                                   facecolor='lightcoral', edgecolor='red',
                                   linewidth=2, alpha=0.7)
        ax.add_patch(dock_patch)
        
        # Add dock labels
        center_x = np.mean(dock_array[:, 0])
        center_y = np.mean(dock_array[:, 1])
        ax.text(center_x, center_y, f'Dock\nArea {i+1}', 
                ha='center', va='center', fontweight='bold', color='darkred')
        
        # Plot vertices
        for x, y in dock:
            ax.plot(x, y, 'ro', markersize=6)
    
    # Plot boundary lines (dock areas 3 and 4)
    # Dock area 3: y position larger than line [-580, 258], [-600, 248]
    line1_x = [-580, -600]
    line1_y = [258, 248]
    ax.plot(line1_x, line1_y, 'r--', linewidth=3, label='Upper boundary (avoid above)')
    
    # Calculate line equation: y = mx + b for upper boundary
    # Slope: m1 = (248 - 258) / (-600 - (-580)) = -10 / -20 = 0.5
    m1 = (248 - 258) / (-600 - (-580))
    b1 = 258 - m1 * (-580)  # y - y1 = m(x - x1)
    
    # Fill area ABOVE this line (avoid area)
    x_fill = np.linspace(-610, -570, 100)
    y_line = m1 * x_fill + b1
    y_upper = np.full_like(x_fill, 270)  # Top of plot area
    ax.fill_between(x_fill, y_line, y_upper, color='lightcoral', alpha=0.3, 
                    label='Avoid above upper line')
    
    # Dock area 4: y position smaller than line [-584, 184], [-593, 183]  
    line2_x = [-584, -593]
    line2_y = [184, 183]
    ax.plot(line2_x, line2_y, 'r--', linewidth=3, label='Lower boundary (avoid below)')
    
    # Calculate line equation for lower boundary
    # Slope: m2 = (183 - 184) / (-593 - (-584)) = -1 / -9 = 1/9
    m2 = (183 - 184) / (-593 - (-584))
    b2 = 184 - m2 * (-584)
    
    # Fill area BELOW this line (avoid area)
    x_fill2 = np.linspace(-610, -570, 100)
    y_line2 = m2 * x_fill2 + b2
    y_lower = np.full_like(x_fill2, 170)  # Bottom of plot area
    ax.fill_between(x_fill2, y_lower, y_line2, color='lightcoral', alpha=0.3,
                    label='Avoid below lower line')
    
    # # Add a sample USV position for reference
    # usv_x, usv_y = -500, 200  # Example USV position
    # ax.plot(usv_x, usv_y, 'bs', markersize=10, label='Sample USV Position')
    # ax.annotate('USV', (usv_x, usv_y), xytext=(10, 10), 
    #             textcoords='offset points', fontsize=12, fontweight='bold')
    
    # # Draw potential paths from USV to each harbor zone
    # for i, zone in enumerate(harbor_zones):
    #     zone_center = np.mean(np.array(zone), axis=0)
    #     ax.plot([usv_x, zone_center[0]], [usv_y, zone_center[1]], 
    #             'b--', alpha=0.5, linewidth=1)
    
    # Formatting
    ax.set_xlim(-620, -560)
    ax.set_ylim(170, 270)
    ax.set_xlabel('X Coordinate (m)', fontsize=12)
    ax.set_ylabel('Y Coordinate (m)', fontsize=12)
    ax.set_title('Harbor Zones and Dock Areas Mapping\n(Green: Safe Harbor Zones, Red: Avoid Dock Areas)', 
                 fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    ax.set_aspect('equal')
    
    # Add coordinate grid for reference
    ax.set_xticks(np.arange(-620, -560, 10))
    ax.set_yticks(np.arange(170, 270, 10))
    
    plt.tight_layout()
    plt.show()
    
    # Print zone information for verification
    print("HARBOR ZONES (Safe Approach Areas):")
    for i, zone in enumerate(harbor_zones):
        print(f"Zone {i+1}: {zone}")
        
    print("\nDOCK AREAS (Avoid These Areas):")
    for i, dock in enumerate(dock_areas):
        print(f"Dock {i+1}: {dock}")
        
    print("\nBOUNDARY LINES:")
    print("Upper boundary: y > line through (-580, 258) and (-600, 248)")
    print("Lower boundary: y < line through (-584, 184) and (-593, 183)")
    
    # Calculate some useful metrics
    print("\nZONE ANALYSIS:")
    for i, zone in enumerate(harbor_zones):
        zone_array = np.array(zone)
        center = np.mean(zone_array, axis=0)
        # Calculate approximate area using shoelace formula
        x = zone_array[:, 0]
        y = zone_array[:, 1]
        area = 0.5 * abs(sum(x[i]*y[i+1] - x[i+1]*y[i] for i in range(-1, len(x)-1)))
        print(f"Zone {i+1}: Center=({center[0]:.1f}, {center[1]:.1f}), Area≈{area:.0f} m²")

if __name__ == "__main__":
    plot_harbor_zones()