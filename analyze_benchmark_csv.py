#!/usr/bin/env python3
"""
Analyze benchmark results from CSV file
For RBE 550 Project 4 - Pendulum Benchmarking
No external dependencies - uses only standard library
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
import sys
from collections import defaultdict

def load_csv(filename):
    """Load benchmark CSV file"""
    data = defaultdict(list)
    
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            planner = row['Planner']
            data[planner].append({
                'run': int(row['Run']),
                'time': float(row['Time']),
                'solved': int(row['Solved']),
                'path_length': float(row['PathLength']) if row['PathLength'] else 0,
                'num_states': int(row['NumStates']) if row['NumStates'] else 0
            })
    
    print(f"Loaded data for planners: {list(data.keys())}")
    total_runs = sum(len(runs) for runs in data.values())
    print(f"Total runs: {total_runs}")
    return data

def print_statistics(data):
    """Print summary statistics for all planners"""
    print("\n" + "="*70)
    print("BENCHMARK STATISTICS SUMMARY")
    print("="*70)
    
    for planner in sorted(data.keys()):
        runs = data[planner]
        total = len(runs)
        solved = sum(r['solved'] for r in runs)
        success_rate = 100 * solved / total if total > 0 else 0
        
        print(f"\n{planner}:")
        print(f"  Runs: {total}")
        print(f"  Success Rate: {success_rate:.1f}% ({solved}/{total})")
        
        if solved > 0:
            successful = [r for r in runs if r['solved'] == 1]
            times = [r['time'] for r in successful]
            lengths = [r['path_length'] for r in successful]
            states = [r['num_states'] for r in successful]
            
            print(f"  Computation Time (successful runs):")
            print(f"    Mean: {np.mean(times):.3f}s")
            print(f"    Median: {np.median(times):.3f}s")
            print(f"    Std: {np.std(times):.3f}s")
            
            print(f"  Path Length:")
            print(f"    Mean: {np.mean(lengths):.3f}")
            print(f"    Median: {np.median(lengths):.3f}")
            print(f"    Std: {np.std(lengths):.3f}")
            
            print(f"  Number of States:")
            print(f"    Mean: {np.mean(states):.1f}")
            print(f"    Median: {np.median(states):.1f}")
            print(f"    Std: {np.std(states):.1f}")
    
    print("\n" + "="*70 + "\n")

def plot_success_rate(data, filename):
    """Plot success rate comparison"""
    plt.figure(figsize=(10, 6))
    
    planners = sorted(data.keys())
    success_rates = []
    for planner in planners:
        runs = data[planner]
        success_rate = 100 * sum(r['solved'] for r in runs) / len(runs)
        success_rates.append(success_rate)
    
    colors = ['#2ecc71', '#3498db', '#e74c3c']
    bars = plt.bar(planners, success_rates, color=colors)
    plt.ylabel('Success Rate (%)', fontsize=12)
    plt.title('Planner Success Rate Comparison', fontsize=14, fontweight='bold')
    plt.ylim([0, 105])
    plt.grid(axis='y', alpha=0.3)
    
    for bar, rate in zip(bars, success_rates):
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height + 1,
                f'{rate:.1f}%', ha='center', va='bottom', fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Saved: {filename}")
    plt.close()

def plot_computation_time(data, filename):
    """Plot computation time comparison"""
    plt.figure(figsize=(10, 6))
    
    planners = sorted(data.keys())
    time_data = []
    labels = []
    
    for planner in planners:
        runs = data[planner]
        successful = [r for r in runs if r['solved'] == 1]
        if successful:
            times = [r['time'] for r in successful]
            time_data.append(times)
            labels.append(planner)
    
    colors = ['#2ecc71', '#3498db', '#e74c3c']
    bp = plt.boxplot(time_data, labels=labels, patch_artist=True)
    
    for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    plt.ylabel('Computation Time (seconds)', fontsize=12)
    plt.title('Planner Computation Time Comparison (Successful Runs)', fontsize=14, fontweight='bold')
    plt.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Saved: {filename}")
    plt.close()

def plot_path_length(data, filename):
    """Plot path length comparison"""
    plt.figure(figsize=(10, 6))
    
    planners = sorted(data.keys())
    length_data = []
    labels = []
    
    for planner in planners:
        runs = data[planner]
        successful = [r for r in runs if r['solved'] == 1]
        if successful:
            lengths = [r['path_length'] for r in successful]
            length_data.append(lengths)
            labels.append(planner)
    
    colors = ['#2ecc71', '#3498db', '#e74c3c']
    bp = plt.boxplot(length_data, labels=labels, patch_artist=True)
    
    for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    plt.ylabel('Path Length', fontsize=12)
    plt.title('Planner Path Length Comparison (Successful Runs)', fontsize=14, fontweight='bold')
    plt.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Saved: {filename}")
    plt.close()

def plot_num_states(data, filename):
    """Plot number of states comparison"""
    plt.figure(figsize=(10, 6))
    
    planners = sorted(data.keys())
    states_data = []
    labels = []
    
    for planner in planners:
        runs = data[planner]
        successful = [r for r in runs if r['solved'] == 1]
        if successful:
            states = [r['num_states'] for r in successful]
            states_data.append(states)
            labels.append(planner)
    
    colors = ['#2ecc71', '#3498db', '#e74c3c']
    bp = plt.boxplot(states_data, labels=labels, patch_artist=True)
    
    for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    plt.ylabel('Number of States', fontsize=12)
    plt.title('Path State Count Comparison', fontsize=14, fontweight='bold')
    plt.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Saved: {filename}")
    plt.close()

def plot_comprehensive(data, filename):
    """Create comprehensive comparison plot"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Comprehensive Planner Comparison', fontsize=16, fontweight='bold')
    
    planners = sorted(data.keys())
    colors = ['#2ecc71', '#3498db', '#e74c3c']
    
    # 1. Success Rate
    ax = axes[0, 0]
    success_rates = []
    for planner in planners:
        runs = data[planner]
        success_rate = 100 * sum(r['solved'] for r in runs) / len(runs)
        success_rates.append(success_rate)
    
    bars = ax.bar(planners, success_rates, color=colors)
    ax.set_ylabel('Success Rate (%)')
    ax.set_title('Success Rate')
    ax.set_ylim([0, 105])
    ax.grid(axis='y', alpha=0.3)
    for bar, rate in zip(bars, success_rates):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 1,
                f'{rate:.1f}%', ha='center', va='bottom', fontsize=9)
    
    # 2. Computation Time
    ax = axes[0, 1]
    time_data = []
    time_labels = []
    for planner in planners:
        runs = data[planner]
        successful = [r for r in runs if r['solved'] == 1]
        if successful:
            times = [r['time'] for r in successful]
            time_data.append(times)
            time_labels.append(planner)
    if time_data:
        bp = ax.boxplot(time_data, labels=time_labels, patch_artist=True)
        for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)
    ax.set_ylabel('Time (seconds)')
    ax.set_title('Computation Time')
    ax.grid(axis='y', alpha=0.3)
    
    # 3. Path Length
    ax = axes[1, 0]
    length_data = []
    length_labels = []
    for planner in planners:
        runs = data[planner]
        successful = [r for r in runs if r['solved'] == 1]
        if successful:
            lengths = [r['path_length'] for r in successful]
            length_data.append(lengths)
            length_labels.append(planner)
    if length_data:
        bp = ax.boxplot(length_data, labels=length_labels, patch_artist=True)
        for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)
    ax.set_ylabel('Path Length')
    ax.set_title('Path Length')
    ax.grid(axis='y', alpha=0.3)
    
    # 4. Number of States
    ax = axes[1, 1]
    states_data = []
    states_labels = []
    for planner in planners:
        runs = data[planner]
        successful = [r for r in runs if r['solved'] == 1]
        if successful:
            states = [r['num_states'] for r in successful]
            states_data.append(states)
            states_labels.append(planner)
    if states_data:
        bp = ax.boxplot(states_data, labels=states_labels, patch_artist=True)
        for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)
    ax.set_ylabel('Number of States')
    ax.set_title('Path States')
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Saved: {filename}")
    plt.close()

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 analyze_benchmark_csv.py <csv_file> <output_prefix>")
        print("Example: python3 analyze_benchmark_csv.py benchmark_results.csv pendulum")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    prefix = sys.argv[2]
    
    print(f"Loading data from {csv_file}...")
    data = load_csv(csv_file)
    
    # Print statistics
    print_statistics(data)
    
    # Generate plots
    print("\nGenerating plots...")
    plot_success_rate(data, f"{prefix}_success_rate.png")
    plot_computation_time(data, f"{prefix}_computation_time.png")
    plot_path_length(data, f"{prefix}_path_length.png")
    plot_num_states(data, f"{prefix}_num_states.png")
    plot_comprehensive(data, f"{prefix}_comprehensive.png")
    
    print("\nAnalysis complete! ✓")
    print(f"Generated 5 PNG files with prefix '{prefix}_'")

if __name__ == "__main__":
    main()