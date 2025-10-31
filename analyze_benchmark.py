# NOT USED DELETE LATER
# 
# 
# #!/usr/bin/env python3
# """
# Analyze OMPL benchmark results from SQLite database
# Generates comparison plots for RRT, EST, and KPIECE1 planners
# """

# import sqlite3
# import numpy as np
# import matplotlib.pyplot as plt
# import sys

# def load_benchmark_data(db_file):
#     """Load benchmark data from OMPL SQLite database"""
#     conn = sqlite3.connect(db_file)
#     cursor = conn.cursor()
    
#     # Get experiment name
#     cursor.execute("SELECT name FROM experiments")
#     experiment_name = cursor.fetchone()[0]
    
#     # Get all planners
#     cursor.execute("SELECT id, name FROM plannerConfigs")
#     planners = {row[0]: row[1] for row in cursor.fetchall()}
    
#     # Get all runs with their data
#     runs_data = {}
#     for planner_id, planner_name in planners.items():
#         cursor.execute("""
#             SELECT time, solved, approximate, length
#             FROM runs
#             WHERE plannerid = ?
#         """, (planner_id,))
        
#         rows = cursor.fetchall()
#         runs_data[planner_name] = {
#             'times': [row[0] for row in rows],
#             'solved': [row[1] for row in rows],
#             'approximate': [row[2] for row in rows],
#             'lengths': [row[3] if row[3] is not None else 0 for row in rows]
#         }
    
#     conn.close()
#     return runs_data, experiment_name

# def get_graph_sizes(db_file, runs_data):
#     """Extract number of nodes (graph size) for each run"""
#     conn = sqlite3.connect(db_file)
#     cursor = conn.cursor()
    
#     # Get planner IDs
#     cursor.execute("SELECT id, name FROM plannerConfigs")
#     planner_ids = {row[1]: row[0] for row in cursor.fetchall()}
    
#     for planner_name in runs_data.keys():
#         planner_id = planner_ids[planner_name]
        
#         # Get all run IDs for this planner
#         cursor.execute("SELECT id FROM runs WHERE plannerid = ?", (planner_id,))
#         run_ids = [row[0] for row in cursor.fetchall()]
        
#         nodes = []
#         for run_id in run_ids:
#             # Get graph size from progress data (last entry)
#             cursor.execute("""
#                 SELECT iterations, motions
#                 FROM progress
#                 WHERE runid = ?
#                 ORDER BY time DESC
#                 LIMIT 1
#             """, (run_id,))
            
#             result = cursor.fetchone()
#             if result:
#                 # Use motions (nodes) if available, otherwise iterations
#                 node_count = result[1] if result[1] is not None else result[0]
#                 nodes.append(node_count if node_count is not None else 0)
#             else:
#                 nodes.append(0)
        
#         runs_data[planner_name]['nodes'] = nodes
    
#     conn.close()
#     return runs_data

# def print_statistics(runs_data):
#     """Print summary statistics for all planners"""
#     print("\n" + "="*70)
#     print("BENCHMARK STATISTICS SUMMARY")
#     print("="*70)
    
#     for planner_name, data in runs_data.items():
#         print(f"\n{planner_name}:")
#         print(f"  Runs: {len(data['times'])}")
        
#         # Success rate
#         success_count = sum(data['solved'])
#         success_rate = 100 * success_count / len(data['solved'])
#         print(f"  Success Rate: {success_rate:.1f}% ({success_count}/{len(data['solved'])})")
        
#         # Computation time (only successful runs)
#         successful_times = [t for t, s in zip(data['times'], data['solved']) if s]
#         if successful_times:
#             print(f"  Computation Time (successful runs):")
#             print(f"    Mean: {np.mean(successful_times):.3f}s")
#             print(f"    Median: {np.median(successful_times):.3f}s")
#             print(f"    Std: {np.std(successful_times):.3f}s")
        
#         # Path length (only successful runs)
#         successful_lengths = [l for l, s in zip(data['lengths'], data['solved']) if s and l > 0]
#         if successful_lengths:
#             print(f"  Path Length (successful runs):")
#             print(f"    Mean: {np.mean(successful_lengths):.3f}")
#             print(f"    Median: {np.median(successful_lengths):.3f}")
#             print(f"    Std: {np.std(successful_lengths):.3f}")
        
#         # Nodes (all runs)
#         if 'nodes' in data and data['nodes']:
#             print(f"  Tree Nodes:")
#             print(f"    Mean: {np.mean(data['nodes']):.1f}")
#             print(f"    Median: {np.median(data['nodes']):.1f}")
#             print(f"    Std: {np.std(data['nodes']):.1f}")
    
#     print("\n" + "="*70 + "\n")

# def plot_success_rate(runs_data, filename):
#     """Plot success rate comparison"""
#     plt.figure(figsize=(10, 6))
    
#     planners = list(runs_data.keys())
#     success_rates = [100 * sum(data['solved']) / len(data['solved']) 
#                      for data in runs_data.values()]
    
#     bars = plt.bar(planners, success_rates, color=['#2ecc71', '#3498db', '#e74c3c'])
#     plt.ylabel('Success Rate (%)', fontsize=12)
#     plt.title('Planner Success Rate Comparison', fontsize=14, fontweight='bold')
#     plt.ylim([0, 105])
#     plt.grid(axis='y', alpha=0.3)
    
#     # Add value labels on bars
#     for bar, rate in zip(bars, success_rates):
#         height = bar.get_height()
#         plt.text(bar.get_x() + bar.get_width()/2., height + 1,
#                 f'{rate:.1f}%', ha='center', va='bottom', fontweight='bold')
    
#     plt.tight_layout()
#     plt.savefig(filename, dpi=300, bbox_inches='tight')
#     print(f"Saved: {filename}")
#     plt.close()

# def plot_computation_time(runs_data, filename):
#     """Plot computation time comparison (box plot)"""
#     plt.figure(figsize=(10, 6))
    
#     # Get successful run times
#     data_to_plot = []
#     labels = []
#     for planner_name, data in runs_data.items():
#         successful_times = [t for t, s in zip(data['times'], data['solved']) if s]
#         if successful_times:
#             data_to_plot.append(successful_times)
#             labels.append(planner_name)
    
#     bp = plt.boxplot(data_to_plot, labels=labels, patch_artist=True)
    
#     # Color the boxes
#     colors = ['#2ecc71', '#3498db', '#e74c3c']
#     for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
#         patch.set_facecolor(color)
#         patch.set_alpha(0.7)
    
#     plt.ylabel('Computation Time (seconds)', fontsize=12)
#     plt.title('Planner Computation Time Comparison (Successful Runs)', fontsize=14, fontweight='bold')
#     plt.grid(axis='y', alpha=0.3)
    
#     plt.tight_layout()
#     plt.savefig(filename, dpi=300, bbox_inches='tight')
#     print(f"Saved: {filename}")
#     plt.close()

# def plot_path_length(runs_data, filename):
#     """Plot path length comparison"""
#     plt.figure(figsize=(10, 6))
    
#     # Get successful run path lengths
#     data_to_plot = []
#     labels = []
#     for planner_name, data in runs_data.items():
#         successful_lengths = [l for l, s in zip(data['lengths'], data['solved']) if s and l > 0]
#         if successful_lengths:
#             data_to_plot.append(successful_lengths)
#             labels.append(planner_name)
    
#     bp = plt.boxplot(data_to_plot, labels=labels, patch_artist=True)
    
#     # Color the boxes
#     colors = ['#2ecc71', '#3498db', '#e74c3c']
#     for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
#         patch.set_facecolor(color)
#         patch.set_alpha(0.7)
    
#     plt.ylabel('Path Length', fontsize=12)
#     plt.title('Planner Path Length Comparison (Successful Runs)', fontsize=14, fontweight='bold')
#     plt.grid(axis='y', alpha=0.3)
    
#     plt.tight_layout()
#     plt.savefig(filename, dpi=300, bbox_inches='tight')
#     print(f"Saved: {filename}")
#     plt.close()

# def plot_tree_nodes(runs_data, filename):
#     """Plot tree nodes comparison"""
#     plt.figure(figsize=(10, 6))
    
#     # Get nodes data
#     data_to_plot = []
#     labels = []
#     for planner_name, data in runs_data.items():
#         if 'nodes' in data and data['nodes']:
#             data_to_plot.append(data['nodes'])
#             labels.append(planner_name)
    
#     if not data_to_plot:
#         print("Warning: No node data available")
#         return
    
#     bp = plt.boxplot(data_to_plot, labels=labels, patch_artist=True)
    
#     # Color the boxes
#     colors = ['#2ecc71', '#3498db', '#e74c3c']
#     for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
#         patch.set_facecolor(color)
#         patch.set_alpha(0.7)
    
#     plt.ylabel('Number of Tree Nodes', fontsize=12)
#     plt.title('Planner Tree Size Comparison', fontsize=14, fontweight='bold')
#     plt.grid(axis='y', alpha=0.3)
    
#     plt.tight_layout()
#     plt.savefig(filename, dpi=300, bbox_inches='tight')
#     print(f"Saved: {filename}")
#     plt.close()

# def plot_comprehensive(runs_data, filename):
#     """Create comprehensive comparison with all metrics"""
#     fig, axes = plt.subplots(2, 2, figsize=(14, 10))
#     fig.suptitle('Comprehensive Planner Comparison', fontsize=16, fontweight='bold')
    
#     planners = list(runs_data.keys())
#     colors = ['#2ecc71', '#3498db', '#e74c3c']
    
#     # 1. Success Rate
#     ax = axes[0, 0]
#     success_rates = [100 * sum(data['solved']) / len(data['solved']) 
#                      for data in runs_data.values()]
#     bars = ax.bar(planners, success_rates, color=colors)
#     ax.set_ylabel('Success Rate (%)')
#     ax.set_title('Success Rate')
#     ax.set_ylim([0, 105])
#     ax.grid(axis='y', alpha=0.3)
#     for bar, rate in zip(bars, success_rates):
#         height = bar.get_height()
#         ax.text(bar.get_x() + bar.get_width()/2., height + 1,
#                 f'{rate:.1f}%', ha='center', va='bottom', fontsize=9)
    
#     # 2. Computation Time
#     ax = axes[0, 1]
#     time_data = []
#     time_labels = []
#     for planner_name, data in runs_data.items():
#         successful_times = [t for t, s in zip(data['times'], data['solved']) if s]
#         if successful_times:
#             time_data.append(successful_times)
#             time_labels.append(planner_name)
#     if time_data:
#         bp = ax.boxplot(time_data, labels=time_labels, patch_artist=True)
#         for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
#             patch.set_facecolor(color)
#             patch.set_alpha(0.7)
#     ax.set_ylabel('Time (seconds)')
#     ax.set_title('Computation Time')
#     ax.grid(axis='y', alpha=0.3)
    
#     # 3. Path Length
#     ax = axes[1, 0]
#     length_data = []
#     length_labels = []
#     for planner_name, data in runs_data.items():
#         successful_lengths = [l for l, s in zip(data['lengths'], data['solved']) if s and l > 0]
#         if successful_lengths:
#             length_data.append(successful_lengths)
#             length_labels.append(planner_name)
#     if length_data:
#         bp = ax.boxplot(length_data, labels=length_labels, patch_artist=True)
#         for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
#             patch.set_facecolor(color)
#             patch.set_alpha(0.7)
#     ax.set_ylabel('Path Length')
#     ax.set_title('Path Length')
#     ax.grid(axis='y', alpha=0.3)
    
#     # 4. Tree Nodes
#     ax = axes[1, 1]
#     node_data = []
#     node_labels = []
#     for planner_name, data in runs_data.items():
#         if 'nodes' in data and data['nodes']:
#             node_data.append(data['nodes'])
#             node_labels.append(planner_name)
#     if node_data:
#         bp = ax.boxplot(node_data, labels=node_labels, patch_artist=True)
#         for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
#             patch.set_facecolor(color)
#             patch.set_alpha(0.7)
#     ax.set_ylabel('Number of Nodes')
#     ax.set_title('Tree Size')
#     ax.grid(axis='y', alpha=0.3)
    
#     plt.tight_layout()
#     plt.savefig(filename, dpi=300, bbox_inches='tight')
#     print(f"Saved: {filename}")
#     plt.close()

# def main():
#     if len(sys.argv) != 3:
#         print("Usage: python3 analyze_benchmark.py <database_file> <output_prefix>")
#         print("Example: python3 analyze_benchmark.py benchmark_pendulum.db pendulum")
#         sys.exit(1)
    
#     db_file = sys.argv[1]
#     prefix = sys.argv[2]
    
#     print(f"Loading benchmark data from {db_file}...")
#     runs_data, experiment_name = load_benchmark_data(db_file)
    
#     print("Extracting tree node data...")
#     runs_data = get_graph_sizes(db_file, runs_data)
    
#     # Print statistics
#     print_statistics(runs_data)
    
#     # Generate plots
#     print("\nGenerating plots...")
#     plot_success_rate(runs_data, f"{prefix}_success_rate.png")
#     plot_computation_time(runs_data, f"{prefix}_computation_time.png")
#     plot_path_length(runs_data, f"{prefix}_path_length.png")
#     plot_tree_nodes(runs_data, f"{prefix}_tree_nodes.png")
#     plot_comprehensive(runs_data, f"{prefix}_comprehensive.png")
    
#     print("\nAnalysis complete! ✓")
#     print(f"Generated 5 PNG files with prefix '{prefix}_'")

# if __name__ == "__main__":
#     main()