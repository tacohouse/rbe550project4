#!/usr/bin/env python3
"""
Car Path Visualizer for RBE 550 Project 4
Visualizes car solution paths with street obstacles
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle as RectPatch
import sys

# Street obstacles (same as in car.cpp)
OBSTACLES = [
    {'x': 5.0, 'y': -2.0, 'width': 7, 'height': 5},
    {'x': -4, 'y': 5, 'width': 16, 'height': 2},
    {'x': -4, 'y': -2, 'width': 7, 'height': 4},
    {'x': 8, 'y': 3, 'width': 4, 'height': 2}
]

START = (1.0, -5.0)
GOAL = (7.0, 4.0)

def load_path(filename):
    """Load car path from file"""
    try:
        data = np.loadtxt(filename)
        
        if data.size == 0:
            print(f"Error: {filename} is empty")
            return None
        
        if len(data.shape) == 1:
            data = data.reshape(1, -1)
        
        x = data[:, 0]
        y = data[:, 1]
        theta = data[:, 2]
        v = data[:, 3]
        
        print(f"Loaded {len(x)} waypoints")
        print(f"X range: [{x.min():.2f}, {x.max():.2f}]")
        print(f"Y range: [{y.min():.2f}, {y.max():.2f}]")
        
        return x, y, theta, v
            
    except Exception as e:
        print(f"Error loading {filename}: {e}")
        return None

def plot_path(x, y, theta, v, planner_name, timeout, save_filename=None):
    """Plot car path with obstacles"""
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    
    # Draw obstacles
    for obs in OBSTACLES:
        rect = RectPatch((obs['x'], obs['y']), obs['width'], obs['height'],
                        linewidth=2, edgecolor='black', facecolor='gray', alpha=0.5)
        ax.add_patch(rect)
    
    # Plot path
    ax.plot(x, y, 'b-', linewidth=2, alpha=0.7, label='Path')
    
    # Mark start and goal
    ax.plot(START[0], START[1], 'go', markersize=15, label='Start', zorder=5)
    ax.plot(GOAL[0], GOAL[1], 'ro', markersize=15, label='Goal', zorder=5)
    
    # Draw arrows showing orientation at some points
    step = max(1, len(x) // 15)  # Show ~15 arrows
    for i in range(0, len(x), step):
        dx = 0.3 * np.cos(theta[i])
        dy = 0.3 * np.sin(theta[i])
        ax.arrow(x[i], y[i], dx, dy, head_width=0.2, head_length=0.15,
                fc='blue', ec='blue', alpha=0.6, zorder=4)
    
    ax.set_xlabel('X (m)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=14, fontweight='bold')
    ax.set_title(f'Car Path - {planner_name} (timeout={timeout}s)\n{len(x)} waypoints',
                fontsize=16, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=12, loc='best')
    ax.axis('equal')
    ax.set_xlim([-5, 13])
    ax.set_ylim([-6, 8])
    
    plt.tight_layout()
    
    if save_filename:
        plt.savefig(save_filename, dpi=300, bbox_inches='tight')
        print(f"Saved plot to {save_filename}")
    else:
        plt.show()
    
    plt.close()

def calculate_path_length(x, y):
    """Calculate total path length"""
    length = 0
    for i in range(1, len(x)):
        dx = x[i] - x[i-1]
        dy = y[i] - y[i-1]
        length += np.sqrt(dx**2 + dy**2)
    return length

def compare_paths(paths_data, save_filename=None):
    """Compare multiple car paths"""
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    
    # Draw obstacles
    for obs in OBSTACLES:
        rect = RectPatch((obs['x'], obs['y']), obs['width'], obs['height'],
                        linewidth=2, edgecolor='black', facecolor='gray', alpha=0.3)
        ax.add_patch(rect)
    
    colors = ['blue', 'green', 'red', 'purple', 'orange']
    
    for i, (x, y, theta, v, planner, timeout) in enumerate(paths_data):
        color = colors[i % len(colors)]
        
        # Plot path
        ax.plot(x, y, '-', color=color, linewidth=2.5, alpha=0.7,
               label=f'{planner} ({timeout}s): {len(x)} pts')
        
        # Start and goal markers
        ax.plot(x[0], y[0], 'o', color=color, markersize=12,
               markeredgecolor='black', markeredgewidth=1.5, zorder=5)
        ax.plot(x[-1], y[-1], 's', color=color, markersize=12,
               markerfacecolor='none', markeredgecolor=color, markeredgewidth=2.5, zorder=5)
    
    # Mark overall start/goal
    ax.plot(START[0], START[1], 'go', markersize=18, label='Start', zorder=6,
           markeredgecolor='black', markeredgewidth=2)
    ax.plot(GOAL[0], GOAL[1], 'ro', markersize=18, label='Goal', zorder=6,
           markeredgecolor='black', markeredgewidth=2)
    
    ax.set_xlabel('X (m)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=14, fontweight='bold')
    ax.set_title('Car Path Comparison', fontsize=16, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=11, loc='best')
    ax.axis('equal')
    ax.set_xlim([-5, 13])
    ax.set_ylim([-6, 8])
    
    plt.tight_layout()
    
    if save_filename:
        plt.savefig(save_filename, dpi=300, bbox_inches='tight')
        print(f"Saved comparison plot to {save_filename}")
    else:
        plt.show()
    
    plt.close()

def main():
    if len(sys.argv) < 2:
        print("Usage: python visualize_car.py <path_file> [planner_name] [timeout]")
        print("   Or: python visualize_car.py compare <file1> <planner1> <timeout1> <file2> ...")
        sys.exit(1)
    
    if sys.argv[1] == "compare":
        if len(sys.argv) < 5 or (len(sys.argv) - 2) % 3 != 0:
            print("Error: For comparison, provide triplets of (filename, planner, timeout)")
            sys.exit(1)
        
        paths_data = []
        i = 2
        while i < len(sys.argv):
            filename = sys.argv[i]
            planner = sys.argv[i+1]
            timeout = sys.argv[i+2]
            
            result = load_path(filename)
            if result is not None:
                x, y, theta, v = result
                paths_data.append((x, y, theta, v, planner, timeout))
                
                path_len = calculate_path_length(x, y)
                print(f"{planner} ({timeout}s): length={path_len:.2f}m, waypoints={len(x)}")
            
            i += 3
        
        if paths_data:
            compare_paths(paths_data, "car_comparison.png")
    
    else:
        filename = sys.argv[1]
        planner = sys.argv[2] if len(sys.argv) > 2 else "Unknown"
        timeout = sys.argv[3] if len(sys.argv) > 3 else "?"
        
        result = load_path(filename)
        if result is not None:
            x, y, theta, v = result
            
            path_len = calculate_path_length(x, y)
            print(f"Path length: {path_len:.2f}m, Waypoints: {len(x)}")
            
            save_name = f"car_{planner}_{timeout}s.png"
            plot_path(x, y, theta, v, planner, timeout, save_name)

if __name__ == "__main__":
    main()