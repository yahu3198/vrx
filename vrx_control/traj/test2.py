import numpy as np
import matplotlib.pyplot as plt

def generate_step_response_test():
    """
    Step response test - best for initial MPC tuning
    Tests: settling time, overshoot, steady-state error
    """
    sample_time = 0.05
    duration = 30
    
    t = np.arange(0, duration + sample_time, sample_time)
    n_points = len(t)
    traj = np.zeros((n_points, 8))  # [x, y, psi, u, v, r, Tp, Ts]
    
    for i, time in enumerate(t):
        if time < 5.0:
            # Phase 1: Stay at origin
            traj[i, :] = [0, 0, 0, 0, 0, 0, 0, 0]
        else:
            # Phase 2: Step to 45 degrees, let MPC determine how to get there
            traj[i, 0] = 0  # x - stay at origin during turn
            traj[i, 1] = 0  # y - stay at origin during turn
            traj[i, 2] = np.pi/4  # psi - 45 degree step
            traj[i, 3] = 0  # u - let MPC decide
            traj[i, 4] = 0  # v - let MPC decide  
            traj[i, 5] = 0  # r - let MPC decide (don't force angular velocity)
            traj[i, 6] = 0  # Tp - let MPC decide
            traj[i, 7] = 0  # Ts - let MPC decide
    
    return t, traj

def generate_ramp_response_test():
    """
    Ramp response test - smooth heading change
    Tests: tracking performance, smooth control
    """
    sample_time = 0.05
    duration = 20
    
    t = np.arange(0, duration + sample_time, sample_time)
    n_points = len(t)
    traj = np.zeros((n_points, 8))
    
    for i, time in enumerate(t):
        if time < 2.0:
            # Initial: straight ahead
            traj[i, :] = [0, 0, 0, 0, 0, 0, 0, 0]
        elif time < 12.0:
            # Ramp turn: 0 to 90 degrees over 10 seconds
            turn_progress = (time - 2.0) / 10.0
            target_heading = np.pi/2 * turn_progress
            
            traj[i, 0] = 0
            traj[i, 1] = 0
            traj[i, 2] = target_heading
            traj[i, 3:] = 0  # Let MPC determine velocities and controls
        else:
            # Hold final heading
            traj[i, 0] = 0
            traj[i, 1] = 0
            traj[i, 2] = np.pi/2  # 90 degrees
            traj[i, 3:] = 0
    
    return t, traj

def generate_realistic_maneuver():
    """
    Realistic maritime maneuver - course change with forward motion
    Tests: Combined position and heading tracking
    """
    sample_time = 0.05
    duration = 40
    
    t = np.arange(0, duration + sample_time, sample_time)
    n_points = len(t)
    traj = np.zeros((n_points, 8))
    
    forward_speed = 0.5  # m/s - realistic USV speed
    
    for i, time in enumerate(t):
        if time < 5.0:
            # Phase 1: Straight line
            traj[i, 0] = forward_speed * time
            traj[i, 1] = 0
            traj[i, 2] = 0
            traj[i, 3] = forward_speed
            traj[i, 4] = 0
            traj[i, 5] = 0
            
        elif time < 20.0:
            # Phase 2: 90 degree turn while maintaining speed
            t_turn = time - 5.0
            turn_duration = 15.0
            turn_progress = min(1.0, t_turn / turn_duration)
            
            # Smooth turn trajectory (quarter circle)
            target_heading = np.pi/2 * turn_progress
            turn_radius = 5.0  # meters
            
            if turn_progress < 1.0:
                # During turn
                angle = target_heading
                traj[i, 0] = 2.5 + turn_radius * np.sin(angle)
                traj[i, 1] = turn_radius * (1 - np.cos(angle))
                traj[i, 2] = target_heading
                traj[i, 3] = forward_speed * np.cos(0)  # Roughly constant speed
                traj[i, 4] = forward_speed * np.sin(0)  # Small lateral component
                traj[i, 5] = (np.pi/2) / turn_duration  # Constant turn rate
            else:
                # Turn complete
                traj[i, 0] = 2.5 + turn_radius
                traj[i, 1] = turn_radius + forward_speed * (t_turn - turn_duration)
                traj[i, 2] = np.pi/2
                traj[i, 3] = 0
                traj[i, 4] = forward_speed
                traj[i, 5] = 0
                
        else:
            # Phase 3: Straight line in new direction
            t_straight = time - 20.0
            traj[i, 0] = 2.5 + turn_radius
            traj[i, 1] = turn_radius + forward_speed * t_straight
            traj[i, 2] = np.pi/2
            traj[i, 3] = 0
            traj[i, 4] = forward_speed
            traj[i, 5] = 0
        
        # Don't specify thrust - let MPC determine
        traj[i, 6] = 0
        traj[i, 7] = 0
    
    return t, traj

def generate_sinusoidal_heading():
    """
    Sinusoidal heading test - continuous turning
    Tests: Frequency response, continuous control
    WARNING: Only use AFTER step response works well
    """
    sample_time = 0.05
    duration = 30
    
    t = np.arange(0, duration + sample_time, sample_time)
    n_points = len(t)
    traj = np.zeros((n_points, 8))
    
    # Low frequency for high-damping system
    freq = 0.05  # Hz (20 second period)
    amplitude = np.pi/6  # ±30 degrees
    
    for i, time in enumerate(t):
        if time < 2.0:
            # Initial steady state
            traj[i, :] = [0, 0, 0, 0, 0, 0, 0, 0]
        else:
            # Sinusoidal heading
            traj[i, 0] = 0
            traj[i, 1] = 0
            traj[i, 2] = amplitude * np.sin(2 * np.pi * freq * (time - 2.0))
            traj[i, 3:] = 0  # Let MPC determine
    
    return t, traj

def plot_trajectories():
    """Visualize all test trajectories"""
    
    # Generate all trajectories
    t1, traj1 = generate_step_response_test()
    t2, traj2 = generate_ramp_response_test()
    t3, traj3 = generate_realistic_maneuver()
    t4, traj4 = generate_sinusoidal_heading()
    
    fig, axes = plt.subplots(2, 4, figsize=(16, 8))
    
    trajectories = [
        (t1, traj1, "Step Response"),
        (t2, traj2, "Ramp Response"),
        (t3, traj3, "Realistic Maneuver"),
        (t4, traj4, "Sinusoidal Heading")
    ]
    
    for col, (t, traj, title) in enumerate(trajectories):
        # Path plot
        axes[0, col].plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2)
        axes[0, col].set_xlabel('X (m)')
        axes[0, col].set_ylabel('Y (m)')
        axes[0, col].set_title(f'{title} - Path')
        axes[0, col].grid(True)
        axes[0, col].axis('equal')
        
        # Heading plot
        axes[1, col].plot(t, traj[:, 2] * 180/np.pi, 'r-', linewidth=2)
        axes[1, col].set_xlabel('Time (s)')
        axes[1, col].set_ylabel('Heading (deg)')
        axes[1, col].set_title(f'{title} - Heading')
        axes[1, col].grid(True)
    
    plt.tight_layout()
    plt.savefig('mpc_test_trajectories.png', dpi=150, bbox_inches='tight')
    plt.show()

def save_trajectories():
    """Save trajectories to files"""
    
    # Generate and save each trajectory
    trajectories = {
        'step_response': generate_step_response_test(),
        'ramp_response': generate_ramp_response_test(), 
        'realistic_maneuver': generate_realistic_maneuver(),
        'sinusoidal_heading': generate_sinusoidal_heading()
    }
    
    for name, (t, traj) in trajectories.items():
        filename = f'trajectory_{name}.txt'
        np.savetxt(filename, traj, fmt='%.6f')
        print(f"Saved {filename}")
    
    return trajectories

if __name__ == '__main__':
    print("=== MPC Test Trajectory Generation ===")
    print()
    
    print("Recommended testing sequence:")
    print("1. Step Response - Test basic settling behavior")
    print("2. Ramp Response - Test smooth tracking")
    print("3. Realistic Maneuver - Test combined motion")
    print("4. Sinusoidal - Test frequency response (advanced)")
    print()
    
    # Save all trajectories
    trajs = save_trajectories()
    
    # Plot for visualization
    plot_trajectories()
    
    print()
    print("Key differences from your original trajectory:")
    print("✅ Bounded heading changes (no infinite growth)")
    print("✅ Realistic angular velocities for high-damping system")
    print("✅ Let MPC determine control inputs (don't force r, Tp, Ts)")
    print("✅ Progressive difficulty for systematic tuning")
    print()
    print("Start with 'trajectory_step_response.txt' for initial MPC tuning!")