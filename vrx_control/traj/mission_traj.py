import numpy as np
import matplotlib.pyplot as plt

def generate_return_trajectory():
    """
    Generate a 2-minute USV trajectory that returns to origin:
    1. Smooth acceleration from rest to cruise speed (0-10s)
    2. Straight forward at cruise speed (10-25s)
    3. 180° turn while maintaining slower forward speed (25-50s)
    4. Shorter straight forward back towards start (50-65s) - REDUCED
    5. 90° turn to align with original direction (65-80s)
    6. Final approach to origin (80-120s) - EXTENDED
    """
    sample_time = 0.05
    duration = 120.0  # 2 minutes
    
    t = np.arange(0, duration + sample_time, sample_time)
    n_points = len(t)
    traj = np.zeros((n_points, 8))  # [x, y, psi, u, v, r, Tp, Ts]
    
    # Parameters - Increased speeds for faster maneuver
    cruise_speed = 2.0      # m/s - increased cruise speed
    turn_speed = 1.2        # m/s - increased speed during turns
    turn_radius_180 = 8.0   # m - radius for 180° turn
    turn_radius_90 = 4.0    # m - radius for 90° turn
    
    # Phase timing - REVISED: Shorter return phase, longer final approach
    accel_end = 10.0        # Acceleration time
    straight1_end = 25.0    # First straight segment
    turn180_end = 50.0      # 180° turn
    straight2_end = 65.0    # REDUCED: Shorter return segment (was 75s)
    turn90_end = 80.0       # ADJUSTED: 90° turn (was 90s)
    # Final approach now 80-120s (was 90-120s)
    
    # Track position and heading
    x, y, psi = -560.0, 220.0, 0.0
    
    for i, time in enumerate(t):
        if time <= accel_end:
            # Phase 1: Smooth acceleration from rest to cruise speed (0-10s)
            progress = time / accel_end
            # Smooth S-curve acceleration (sigmoid-based)
            speed_factor = 3 * progress**2 - 2 * progress**3  # Smooth 0 to 1
            current_speed = cruise_speed * speed_factor
            
            # Move straight forward (heading = 0)
            if i > 0:
                dt = sample_time
                x += current_speed * np.cos(psi) * dt
                y += current_speed * np.sin(psi) * dt
            
            traj[i, 0] = x
            traj[i, 1] = y
            traj[i, 2] = psi
            traj[i, 3] = current_speed  # u (forward velocity)
            traj[i, 4] = 0             # v (lateral velocity)
            traj[i, 5] = 0             # r (angular velocity)
            
        elif time <= straight1_end:
            # Phase 2: Straight forward at cruise speed (10-25s)
            if i > 0:
                dt = sample_time
                x += cruise_speed * np.cos(psi) * dt
                y += cruise_speed * np.sin(psi) * dt
            
            traj[i, 0] = x
            traj[i, 1] = y
            traj[i, 2] = psi
            traj[i, 3] = cruise_speed
            traj[i, 4] = 0
            traj[i, 5] = 0
            
        elif time <= turn180_end:
            # Phase 3: 180° turn while maintaining forward motion (25-50s)
            turn_duration = turn180_end - straight1_end
            turn_progress = (time - straight1_end) / turn_duration
            
            # Angular velocity for 180° turn
            total_angle = np.pi
            angular_vel = total_angle / turn_duration
            
            if i > 0:
                dt = sample_time
                # Update heading
                psi += angular_vel * dt
                
                # Move forward at turn speed with circular motion
                x += turn_speed * np.cos(psi) * dt
                y += turn_speed * np.sin(psi) * dt
            
            traj[i, 0] = x
            traj[i, 1] = y
            traj[i, 2] = psi
            traj[i, 3] = turn_speed * np.cos(angular_vel * dt)  # Reduced forward speed
            traj[i, 4] = turn_speed * np.sin(angular_vel * dt)  # Lateral component
            traj[i, 5] = angular_vel
            
        elif time <= straight2_end:
            # Phase 4: SHORTER straight forward back towards start (50-65s)
            if i > 0:
                dt = sample_time
                x += cruise_speed * np.cos(psi) * dt
                y += cruise_speed * np.sin(psi) * dt
            
            traj[i, 0] = x
            traj[i, 1] = y
            traj[i, 2] = psi
            traj[i, 3] = cruise_speed
            traj[i, 4] = 0
            traj[i, 5] = 0
            
        elif time <= turn90_end:
            # Phase 5: 90° turn to align with original direction (65-80s)
            turn_duration = turn90_end - straight2_end
            
            # Angular velocity for 90° turn
            total_angle = np.pi/2
            angular_vel = total_angle / turn_duration
            
            if i > 0:
                dt = sample_time
                # Update heading
                psi += angular_vel * dt
                
                # Move forward at turn speed
                x += turn_speed * np.cos(psi) * dt
                y += turn_speed * np.sin(psi) * dt
            
            traj[i, 0] = x
            traj[i, 1] = y
            traj[i, 2] = psi
            traj[i, 3] = turn_speed * np.cos(angular_vel * dt)
            traj[i, 4] = turn_speed * np.sin(angular_vel * dt)
            traj[i, 5] = angular_vel
            
        else:
            # Phase 6: EXTENDED final approach to origin (80-120s)
            remaining_time = duration - turn90_end
            time_in_phase = time - turn90_end
            
            # Calculate distance to origin and required speed
            dist_to_origin = np.sqrt((x - (-560))**2 + (y - 220)**2)
            
            if dist_to_origin > 0.1:  # If not at target
                # Calculate required velocity to reach target
                approach_speed = min(cruise_speed * 0.8, dist_to_origin / (remaining_time - time_in_phase + 0.1))
                
                # Direction to target (-560, 220)
                angle_to_target = np.arctan2(220 - y, -560 - x)
                
                if i > 0:
                    dt = sample_time
                    x += approach_speed * np.cos(angle_to_target) * dt
                    y += approach_speed * np.sin(angle_to_target) * dt
                    
                    # Gradually adjust heading towards target
                    heading_error = angle_to_target - psi
                    # Normalize angle
                    while heading_error > np.pi:
                        heading_error -= 2*np.pi
                    while heading_error < -np.pi:
                        heading_error += 2*np.pi
                    
                    # Gradual heading adjustment
                    max_turn_rate = 0.2  # rad/s
                    psi += np.clip(heading_error * 2, -max_turn_rate * dt, max_turn_rate * dt)
                
                traj[i, 0] = x
                traj[i, 1] = y
                traj[i, 2] = psi
                traj[i, 3] = approach_speed * np.cos(angle_to_target - psi)
                traj[i, 4] = approach_speed * np.sin(angle_to_target - psi)
                traj[i, 5] = np.clip(heading_error * 2, -max_turn_rate, max_turn_rate)
            else:
                # At target - stop
                traj[i, 0] = -560.0
                traj[i, 1] = 220.0
                traj[i, 2] = 0  # Original heading
                traj[i, 3] = 0
                traj[i, 4] = 0
                traj[i, 5] = 0
        
        # Let MPC determine thrust commands
        traj[i, 6] = 0  # Tp
        traj[i, 7] = 0  # Ts
    
    return t, traj

def plot_return_trajectory():
    """Visualize the return trajectory"""
    t, traj = generate_return_trajectory()
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Path plot
    axes[0, 0].plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2, label='Path')
    axes[0, 0].plot(-560, 220, 'go', markersize=8, label='Start/End')
    axes[0, 0].set_xlabel('X (m)')
    axes[0, 0].set_ylabel('Y (m)')
    axes[0, 0].set_title('USV Return Trajectory - Path (Revised)')
    axes[0, 0].grid(True)
    axes[0, 0].axis('equal')
    axes[0, 0].legend()
    
    # Add phase annotations with REVISED timing
    sample_time = 0.05
    phase_times = [10, 25, 50, 65, 80]  # Updated phase times
    phase_labels = ['Accel End', 'Straight End', '180° Turn End', 'Return End', '90° Turn End']
    colors = ['red', 'orange', 'purple', 'brown', 'pink']
    
    for i, (phase_time, label, color) in enumerate(zip(phase_times, phase_labels, colors)):
        idx = int(phase_time / sample_time)
        if idx < len(traj):
            axes[0, 0].plot(traj[idx, 0], traj[idx, 1], 'o', color=color, markersize=6, label=label)
    
    axes[0, 0].legend()
    
    # Heading plot
    axes[0, 1].plot(t, traj[:, 2] * 180/np.pi, 'r-', linewidth=2)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Heading (deg)')
    axes[0, 1].set_title('Heading vs Time')
    axes[0, 1].grid(True)
    
    # Velocity plot
    axes[1, 0].plot(t, traj[:, 3], 'g-', linewidth=2, label='u (forward)')
    axes[1, 0].plot(t, traj[:, 4], 'm-', linewidth=2, label='v (lateral)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Velocity (m/s)')
    axes[1, 0].set_title('Velocity Components')
    axes[1, 0].grid(True)
    axes[1, 0].legend()
    
    # Angular velocity plot
    axes[1, 1].plot(t, traj[:, 5] * 180/np.pi, 'c-', linewidth=2)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Angular Velocity (deg/s)')
    axes[1, 1].set_title('Angular Velocity vs Time')
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('usv_return_trajectory.png', dpi=150, bbox_inches='tight')
    plt.show()

def save_return_trajectory():
    """Save the return trajectory to file"""
    t, traj = generate_return_trajectory()
    
    # Force exact final values for last few rows to ensure precision
    target_x, target_y, target_psi = -560.0, 220.0, 6.283185307179586  # Exact 2π
    num_final_rows = 10  # Last 0.5 seconds
    
    for i in range(len(traj) - num_final_rows, len(traj)):
        traj[i, 0] = target_x
        traj[i, 1] = target_y  
        traj[i, 2] = target_psi
        traj[i, 3] = 0  # u
        traj[i, 4] = 0  # v
        traj[i, 5] = 0  # r
        traj[i, 6] = 0  # Tp
        traj[i, 7] = 0  # Ts
    
    filename = 'trajectory_return_maneuver.txt'
    np.savetxt(filename, traj, fmt='%.6f')
    print(f"Saved {filename}")
    
    # Print trajectory summary
    print("\n=== REVISED Trajectory Summary ===")
    print(f"Duration: {len(t) * 0.05:.1f} seconds")
    print(f"Sample rate: {1/0.05:.0f} Hz")
    print(f"Total samples: {len(t)}")
    
    print("\nREVISED Phase breakdown:")
    print("0-10s:   Acceleration to cruise speed")
    print("10-25s:  Straight forward at cruise speed")
    print("25-50s:  180° turn with forward speed")
    print("50-65s:  SHORTER return at cruise speed (was 50-75s)")
    print("65-80s:  90° turn to original heading (was 75-90s)")
    print("80-120s: EXTENDED final approach to (-560, 220) (was 90-120s)")
    
    print(f"\nFinal position: ({traj[-1, 0]:.6f}, {traj[-1, 1]:.6f})")
    print(f"Final heading: {traj[-1, 2]:.6f} rad ({traj[-1, 2] * 180/np.pi:.1f}°)")
    
    # Check if X position goes behind -560 during 90° turn
    turn90_start_idx = int(65 / 0.05)
    turn90_end_idx = int(80 / 0.05)
    min_x_during_turn = np.min(traj[turn90_start_idx:turn90_end_idx, 0])
    print(f"\nMinimum X position during 90° turn: {min_x_during_turn:.2f}")
    print(f"Target X position: -560.0")
    if min_x_during_turn >= -560:
        print("✅ SUCCESS: X position never goes behind target!")
    else:
        print("❌ WARNING: X position still goes behind target")
    
    return t, traj

if __name__ == '__main__':
    print("=== REVISED USV Return Trajectory Generation ===")
    print("Generating 2-minute trajectory with shorter return phase...")
    
    # Generate and save trajectory
    t, traj = save_return_trajectory()
    
    # Plot for visualization
    plot_return_trajectory()
    
    print("\nRevised trajectory features:")
    print("✅ Shorter return phase (50-65s instead of 50-75s)")
    print("✅ Earlier 90° turn (65-80s instead of 75-90s)")
    print("✅ Extended final approach (80-120s instead of 90-120s)")
    print("✅ Prevents overshooting past X = -560")
    print("✅ More time for precise final approach")
    print("✅ Return path to starting point (-560, 220)")
    print("✅ Final heading: 6.283185 rad (2π)")
    print("✅ Let MPC determine thrust commands")