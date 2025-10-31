#!/usr/bin/env python3
"""
///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Luis Alzamora Josh Ethan
//////////////////////////////////////
"""

import numpy as np
import matplotlib.pyplot as plt
import sys

def wrap_angle(theta):
    """Wrap angle to [-pi, pi] range"""
    return np.arctan2(np.sin(theta), np.cos(theta))

def load_path(filename):
    """Load path data from file"""
    try:
        data = np.loadtxt(filename)
        
        if data.size == 0:
            print(f"Error: {filename} is empty")
            return None
        
        if len(data.shape) == 1:
            data = data.reshape(1, -1)
        
        theta = data[:, 0]
        omega = data[:, 1]
        
        print(f"Loaded {len(theta)} waypoints")
        print(f"Theta range (before wrapping): [{theta.min():.3f}, {theta.max():.3f}]")
        print(f"Omega range: [{omega.min():.3f}, {omega.max():.3f}]")
        
        return theta, omega
            
    except Exception as e:
        print(f"Error loading {filename}: {e}")
        return None

def plot_path(theta, omega, torque, planner_name, save_filename=None):
    """Plot path in phase space with angle wrapping"""
    
    # Wrap all theta values to [-pi, pi]
    theta_wrapped = wrap_angle(theta)
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    
    # Split into segments at wraparound discontinuities
    segments_theta = []
    segments_omega = []
    current_theta = [theta_wrapped[0]]
    current_omega = [omega[0]]
    
    for i in range(1, len(theta_wrapped)):
        # Detect large jump (wraparound)
        if abs(theta_wrapped[i] - theta_wrapped[i-1]) > np.pi/2:
            # Save current segment
            segments_theta.append(np.array(current_theta))
            segments_omega.append(np.array(current_omega))
            # Start new segment
            current_theta = [theta_wrapped[i]]
            current_omega = [omega[i]]
        else:
            current_theta.append(theta_wrapped[i])
            current_omega.append(omega[i])
    
    # Add last segment
    segments_theta.append(np.array(current_theta))
    segments_omega.append(np.array(current_omega))
    
    # Plot each segment separately
    for seg_theta, seg_omega in zip(segments_theta, segments_omega):
        ax.plot(seg_theta, seg_omega, 'b-', linewidth=2, alpha=0.7)
    
    # Add label for legend
    ax.plot([], [], 'b-', linewidth=2, alpha=0.7, label='Trajectory')
    
    # Mark start and goal
    ax.plot(theta_wrapped[0], omega[0], 'go', markersize=15, label='Start', zorder=5)
    ax.plot(theta_wrapped[-1], omega[-1], 'ro', markersize=15, label='Goal', zorder=5)
    
    # Reference lines
    ax.axvline(x=-np.pi/2, color='g', linestyle='--', alpha=0.3, linewidth=2, label='Start θ = -π/2')
    ax.axvline(x=np.pi/2, color='r', linestyle='--', alpha=0.3, linewidth=2, label='Goal θ = π/2')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3, linewidth=1.5)
    ax.axhline(y=10, color='orange', linestyle=':', alpha=0.5, linewidth=2, label='ω bounds = ±10')
    ax.axhline(y=-10, color='orange', linestyle=':', alpha=0.5, linewidth=2)
    
    ax.set_xlabel('θ (rad)', fontsize=16, fontweight='bold')
    ax.set_ylabel('ω (rad/s)', fontsize=16, fontweight='bold')
    ax.set_title(f'Phase Space Trajectory - {planner_name}\nTorque = {torque}', fontsize=18, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=12, loc='best')
    
    ax.set_xlim([-np.pi-0.2, np.pi+0.2])
    ax.set_ylim([-12, 12])
    ax.set_xticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi])
    ax.set_xticklabels(['-π', '-π/2', '0', 'π/2', 'π'])
    
    plt.tight_layout()
    
    if save_filename:
        plt.savefig(save_filename, dpi=300, bbox_inches='tight')
        print(f"Saved plot to {save_filename}")
    else:
        plt.show()
    
    plt.close()

def compare_paths(paths_data, save_filename=None):
    """Compare multiple paths on one plot"""
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    
    colors = ['blue', 'green', 'red', 'purple', 'orange', 'brown']
    markers = ['o', 's', '^', 'D', 'v', '<']
    
    for i, (theta, omega, torque, planner) in enumerate(paths_data):
        color = colors[i % len(colors)]
        marker = markers[i % len(markers)]
        
        # Wrap theta
        theta_wrapped = wrap_angle(theta)
        
        # Split into segments at wraparound discontinuities
        segments_theta = []
        segments_omega = []
        current_theta = [theta_wrapped[0]]
        current_omega = [omega[0]]
        
        for j in range(1, len(theta_wrapped)):
            if abs(theta_wrapped[j] - theta_wrapped[j-1]) > np.pi/2:
                segments_theta.append(np.array(current_theta))
                segments_omega.append(np.array(current_omega))
                current_theta = [theta_wrapped[j]]
                current_omega = [omega[j]]
            else:
                current_theta.append(theta_wrapped[j])
                current_omega.append(omega[j])
        
        segments_theta.append(np.array(current_theta))
        segments_omega.append(np.array(current_omega))
        
        # Plot each segment
        for seg_theta, seg_omega in zip(segments_theta, segments_omega):
            ax.plot(seg_theta, seg_omega, '-', color=color, linewidth=2.5, alpha=0.7)
        
        # Add label
        ax.plot([], [], '-', color=color, linewidth=2.5, label=f'{planner}, τ={torque}', alpha=0.7)
        
        ax.plot(theta_wrapped[0], omega[0], marker, color=color, markersize=14, 
                markeredgecolor='black', markeredgewidth=1.5, zorder=5)
        ax.plot(theta_wrapped[-1], omega[-1], marker, color=color, markersize=14, 
                markerfacecolor='none', markeredgecolor=color, markeredgewidth=2.5, zorder=5)
    
    # Reference lines
    ax.axvline(x=-np.pi/2, color='g', linestyle='--', alpha=0.3, linewidth=2, label='Start θ = -π/2')
    ax.axvline(x=np.pi/2, color='r', linestyle='--', alpha=0.3, linewidth=2, label='Goal θ = π/2')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3, linewidth=1.5)
    ax.axhline(y=10, color='orange', linestyle=':', alpha=0.3, linewidth=2, label='ω bounds')
    ax.axhline(y=-10, color='orange', linestyle=':', alpha=0.3, linewidth=2)
    
    ax.set_xlabel('θ (rad)', fontsize=16, fontweight='bold')
    ax.set_ylabel('ω (rad/s)', fontsize=16, fontweight='bold')
    ax.set_title('Pendulum Trajectory Comparison', fontsize=18, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=12, loc='best')
    
    ax.set_xlim([-np.pi-0.2, np.pi+0.2])
    ax.set_ylim([-12, 12])
    ax.set_xticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi])
    ax.set_xticklabels(['-π', '-π/2', '0', 'π/2', 'π'])
    
    plt.tight_layout()
    
    if save_filename:
        plt.savefig(save_filename, dpi=300, bbox_inches='tight')
        print(f"Saved comparison plot to {save_filename}")
    else:
        plt.show()
    
    plt.close()

def calculate_metrics(theta, omega):
    """Calculate path metrics"""
    path_length = 0
    for i in range(1, len(theta)):
        dtheta = theta[i] - theta[i-1]
        domega = omega[i] - omega[i-1]
        path_length += np.sqrt(dtheta**2 + domega**2)
    
    return path_length, len(theta)

def main():
    if len(sys.argv) < 2:
        print("Usage: python visualize_pendulum.py <path_file> [torque] [planner_name]")
        print("   Or: python visualize_pendulum.py compare <file1> <torque1> <planner1> <file2> <torque2> <planner2> ...")
        sys.exit(1)
    
    if sys.argv[1] == "compare":
        if len(sys.argv) < 5 or (len(sys.argv) - 2) % 3 != 0:
            print("Error: For comparison, provide triplets of (filename, torque, planner)")
            sys.exit(1)
        
        paths_data = []
        i = 2
        while i < len(sys.argv):
            filename = sys.argv[i]
            torque = sys.argv[i+1]
            planner = sys.argv[i+2]
            
            result = load_path(filename)
            if result is not None:
                theta, omega = result
                paths_data.append((theta, omega, torque, planner))
                
                path_len, num_pts = calculate_metrics(theta, omega)
                print(f"{planner} (Torque={torque}): length={path_len:.3f}, waypoints={num_pts}")
            
            i += 3
        
        if paths_data:
            compare_paths(paths_data, "pendulum_comparison.png")
    
    else:
        filename = sys.argv[1]
        torque = sys.argv[2] if len(sys.argv) > 2 else "?"
        planner = sys.argv[3] if len(sys.argv) > 3 else "Unknown"
        
        result = load_path(filename)
        if result is not None:
            theta, omega = result
            
            path_len, num_pts = calculate_metrics(theta, omega)
            print(f"Path length: {path_len:.3f}, Waypoints: {num_pts}")
            
            save_name = f"pendulum_torque{torque}_{planner}.png"
            plot_path(theta, omega, torque, planner, save_name)

if __name__ == "__main__":
    main()
