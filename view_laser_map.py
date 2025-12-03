# view_laser_map.py
# Visualization tool for saved laser mapping data
# Loads .npz files created by gantry_laser_mapper.py or gantry_laser_mapper_v2.py
# 
# Height formula used: actual_height = -((-gantry_z) - laser_z)

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.colors import Normalize
import argparse
from pathlib import Path


def calculate_actual_height(gantry_z, laser_z):
    """
    Calculate actual object height from gantry Z and laser distance.
    Consistent with gantry_laser_mapper_v2.py formula.
    """
    return -((-gantry_z * 1000) - laser_z)  # Returns mm


def load_map_data(filename):
    """Load mapping data from .npz file"""
    print(f"Loading {filename}...")
    data = np.load(filename, allow_pickle=True)
    
    print("\nData contents:")
    for key in data.files:
        arr = data[key]
        if isinstance(arr, np.ndarray):
            if arr.dtype == object:
                print(f"  {key}: {len(arr)} arrays (object dtype)")
            else:
                print(f"  {key}: shape={arr.shape}, dtype={arr.dtype}")
        else:
            print(f"  {key}: {arr}")
    
    return data


def plot_height_map(data, cmap='viridis', point_size=1):
    """Plot the full height map as a scatter plot"""
    # Support both v1 and v2 formats
    if 'full_x' in data.files:
        # V2 format
        map_x = data['full_x']
        map_y = data['full_y']
        map_height = data['full_height']
    elif 'map_x' in data.files:
        # V1 format
        map_x = data['map_x']
        map_y = data['map_y']
        map_height = data['map_height']
    else:
        print("No map data available!")
        return
    
    if len(map_x) == 0:
        print("No map data available!")
        return
    
    fig, ax = plt.subplots(figsize=(12, 10), facecolor='white')
    
    # Filter valid data
    valid = np.isfinite(map_height)
    x = map_x[valid]
    y = map_y[valid]
    h = map_height[valid]
    
    # Create scatter plot
    scatter = ax.scatter(x, y, c=h, cmap=cmap, s=point_size, marker='s')
    
    # Colorbar
    cbar = fig.colorbar(scatter, ax=ax, label='Height (mm)')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Height Map - {len(x):,} points')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    # Stats
    stats_text = f"Height range: {h.min():.1f} - {h.max():.1f} mm\n"
    stats_text += f"X range: {x.min():.3f} - {x.max():.3f} m\n"
    stats_text += f"Y range: {y.min():.3f} - {y.max():.3f} m"
    ax.text(0.02, 0.02, stats_text, transform=ax.transAxes,
            fontsize=9, verticalalignment='bottom',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    return fig, ax


def plot_height_map_gridded(data, grid_resolution=0.001, cmap='viridis'):
    """Plot height map as a gridded image (faster for large datasets)"""
    # Support both v1 and v2 formats
    if 'avg_x' in data.files:
        # V2 averaged format - use directly
        print("Using pre-averaged data...")
        map_x = data['avg_x']
        map_y = data['avg_y']
        map_height = data['avg_height']
    elif 'full_x' in data.files:
        # V2 full format
        map_x = data['full_x']
        map_y = data['full_y']
        map_height = data['full_height']
    elif 'map_x' in data.files:
        # V1 format
        map_x = data['map_x']
        map_y = data['map_y']
        map_height = data['map_height']
    else:
        print("No map data available!")
        return
    
    if len(map_x) == 0:
        print("No map data available!")
        return
    
    # Filter valid data
    valid = np.isfinite(map_height)
    x = map_x[valid]
    y = map_y[valid]
    h = map_height[valid]
    
    # Create grid
    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()
    
    nx = int((x_max - x_min) / grid_resolution) + 1
    ny = int((y_max - y_min) / grid_resolution) + 1
    
    # Limit for memory
    max_grid = 2000
    if nx > max_grid or ny > max_grid:
        scale = max(nx, ny) / max_grid
        grid_resolution *= scale
        nx = int((x_max - x_min) / grid_resolution) + 1
        ny = int((y_max - y_min) / grid_resolution) + 1
    
    print(f"Creating {nx}x{ny} grid (resolution: {grid_resolution*1000:.2f}mm)...")
    
    # Grid the data
    grid = np.full((ny, nx), np.nan)
    count = np.zeros((ny, nx))
    
    # Convert coordinates to grid indices
    xi = ((x - x_min) / grid_resolution).astype(int)
    yi = ((y - y_min) / grid_resolution).astype(int)
    
    # Clip to valid range
    xi = np.clip(xi, 0, nx - 1)
    yi = np.clip(yi, 0, ny - 1)
    
    # Accumulate (average multiple points in same cell)
    for i in range(len(x)):
        if np.isfinite(h[i]):
            if np.isnan(grid[yi[i], xi[i]]):
                grid[yi[i], xi[i]] = h[i]
                count[yi[i], xi[i]] = 1
            else:
                grid[yi[i], xi[i]] += h[i]
                count[yi[i], xi[i]] += 1
    
    # Average
    mask = count > 0
    grid[mask] /= count[mask]
    
    # Plot
    fig, ax = plt.subplots(figsize=(12, 10), facecolor='white')
    
    extent = [x_min, x_max, y_min, y_max]
    im = ax.imshow(grid, origin='lower', extent=extent, aspect='equal',
                   cmap=cmap, interpolation='nearest')
    
    cbar = fig.colorbar(im, ax=ax, label='Height (mm)')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Height Map (Gridded) - Resolution: {grid_resolution*1000:.2f}mm')
    ax.grid(True, alpha=0.3)
    
    # Stats
    valid_grid = grid[~np.isnan(grid)]
    if len(valid_grid) > 0:
        stats_text = f"Height range: {valid_grid.min():.1f} - {valid_grid.max():.1f} mm\n"
        stats_text += f"Grid cells filled: {np.sum(mask):,} / {nx*ny:,}"
        ax.text(0.02, 0.02, stats_text, transform=ax.transAxes,
                fontsize=9, verticalalignment='bottom',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    return fig, ax, grid


def plot_scan_trajectory(data):
    """Plot the scanning trajectory (gantry path)"""
    # Support both formats
    if 'scan_gantry_x' in data.files:
        # V2 format
        gantry_x = data['scan_gantry_x']
        gantry_y = data['scan_gantry_y']
        gantry_z = data['scan_gantry_z']
        timestamps = data['scan_timestamps']
    elif 'gantry_x' in data.files:
        # V1 format
        gantry_x = data['gantry_x']
        gantry_y = data['gantry_y']
        gantry_z = data['gantry_z']
        timestamps = data['timestamps']
    else:
        print("No trajectory data!")
        return
    
    if len(timestamps) == 0:
        print("No trajectory data!")
        return
    
    # Time in seconds from start
    t = timestamps - timestamps[0]
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10), facecolor='white')
    
    # XY trajectory
    ax = axes[0, 0]
    ax.plot(gantry_x, gantry_y, 'b-', lw=0.5, alpha=0.7)
    ax.plot(gantry_x[0], gantry_y[0], 'go', markersize=10, label='Start')
    ax.plot(gantry_x[-1], gantry_y[-1], 'ro', markersize=10, label='End')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Scanning Trajectory (XY)')
    ax.set_aspect('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # X vs time
    ax = axes[0, 1]
    ax.plot(t, gantry_x, 'b-', lw=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('X (m)')
    ax.set_title('X Position vs Time')
    ax.grid(True, alpha=0.3)
    
    # Y vs time
    ax = axes[1, 0]
    ax.plot(t, gantry_y, 'g-', lw=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Y Position vs Time')
    ax.grid(True, alpha=0.3)
    
    # Z vs time
    ax = axes[1, 1]
    ax.plot(t, gantry_z, 'r-', lw=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Z (m)')
    ax.set_title('Z (Laser Height) vs Time')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def plot_3d_surface(data, downsample=10, cmap='viridis'):
    """Plot 3D surface view of the height map"""
    # Support both formats
    if 'full_x' in data.files:
        map_x = data['full_x']
        map_y = data['full_y']
        map_height = data['full_height']
    elif 'avg_x' in data.files:
        map_x = data['avg_x']
        map_y = data['avg_y']
        map_height = data['avg_height']
        downsample = 1  # Already averaged
    elif 'map_x' in data.files:
        map_x = data['map_x']
        map_y = data['map_y']
        map_height = data['map_height']
    else:
        print("No map data!")
        return
    
    if len(map_x) == 0:
        print("No map data!")
        return
    
    # Filter and downsample
    valid = np.isfinite(map_height)
    x = map_x[valid][::downsample]
    y = map_y[valid][::downsample]
    h = map_height[valid][::downsample]
    
    fig = plt.figure(figsize=(12, 9), facecolor='white')
    ax = fig.add_subplot(111, projection='3d')
    
    # Convert height to meters for z-axis if in mm
    h_plot = h / 1000.0 if np.nanmax(np.abs(h)) > 10 else h
    
    scatter = ax.scatter(x, y, h_plot, c=h, cmap=cmap, s=1, alpha=0.8)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Height (m)')
    ax.set_title(f'3D Height Map ({len(x):,} points)')
    
    fig.colorbar(scatter, ax=ax, label='Height (mm)', shrink=0.6)
    
    plt.tight_layout()
    return fig, ax


def interactive_scan_viewer(data):
    """Interactive viewer to browse individual scans"""
    # Check for V2 format first (doesn't store individual laser arrays in same way)
    if 'laser_x' not in data.files:
        print("Interactive scan viewer not available for V2 format.")
        print("Use --grid or default view for V2 data.")
        return None, None
    
    laser_x = data['laser_x']
    laser_z = data['laser_z']
    
    # Support both formats for gantry data
    if 'scan_gantry_x' in data.files:
        gantry_x = data['scan_gantry_x']
        gantry_y = data['scan_gantry_y']
        gantry_z = data['scan_gantry_z']
        timestamps = data['scan_timestamps']
    else:
        gantry_x = data['gantry_x']
        gantry_y = data['gantry_y']
        gantry_z = data['gantry_z']
        timestamps = data['timestamps']
    
    n_scans = len(timestamps)
    if n_scans == 0:
        print("No scan data!")
        return None, None
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), facecolor='white')
    
    # Left: laser profile
    ax1.set_xlabel('Laser X (mm)')
    ax1.set_ylabel('Laser Z (mm)')
    ax1.set_title('Laser Profile')
    ax1.grid(True, alpha=0.3)
    line, = ax1.plot([], [], 'g-', lw=1)
    center_point, = ax1.plot([], [], 'ro', markersize=8)
    
    # Right: trajectory with current position
    ax2.plot(gantry_x, gantry_y, 'b-', lw=0.5, alpha=0.5, label='Path')
    current_pos, = ax2.plot([], [], 'ro', markersize=10, label='Current')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Gantry Position')
    ax2.set_aspect('equal')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Info text
    info_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes,
                         verticalalignment='top', fontsize=9,
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # Slider
    plt.subplots_adjust(bottom=0.2)
    ax_slider = plt.axes([0.2, 0.05, 0.6, 0.03])
    slider = Slider(ax_slider, 'Scan', 0, n_scans - 1, valinit=0, valstep=1)
    
    def update(val):
        idx = int(slider.val)
        
        # Get laser data for this scan
        lx = laser_x[idx]
        lz = laser_z[idx]
        
        valid = np.isfinite(lz)
        if np.any(valid):
            line.set_data(lx[valid], lz[valid])
            
            # Auto-scale
            z_valid = lz[valid]
            ax1.set_xlim(lx.min() - 5, lx.max() + 5)
            ax1.set_ylim(z_valid.min() - 10, z_valid.max() + 10)
            
            # Center point
            center_idx = len(lz) // 2
            if np.isfinite(lz[center_idx]):
                center_point.set_data([lx[center_idx]], [lz[center_idx]])
        
        # Update position marker
        current_pos.set_data([gantry_x[idx]], [gantry_y[idx]])
        
        # Update info - using correct height formula
        t = timestamps[idx] - timestamps[0]
        gz = gantry_z[idx]
        center_lz = lz[len(lz)//2] if np.isfinite(lz[len(lz)//2]) else np.nan
        height = calculate_actual_height(gz, center_lz) if np.isfinite(center_lz) else np.nan
        
        info = f"Scan {idx + 1}/{n_scans}\n"
        info += f"Time: {t:.2f}s\n"
        info += f"Gantry: X={gantry_x[idx]:.3f} Y={gantry_y[idx]:.3f} Z={gz:.3f}\n"
        if np.isfinite(height):
            info += f"Center Height: {height:.1f} mm"
        info_text.set_text(info)
        
        fig.canvas.draw_idle()
    
    slider.on_changed(update)
    update(0)
    
    plt.tight_layout()
    return fig, slider


def analyze_data(data):
    """Print analysis of the mapping data"""
    print("\n" + "="*60)
    print("DATA ANALYSIS")
    print("="*60)
    
    if 'scan_count' in data.files:
        print(f"Total scans: {data['scan_count']}")
    elif 'total_scans' in data.files:
        print(f"Total scans: {data['total_scans']}")
    
    if 'point_count' in data.files:
        print(f"Total points: {int(data['point_count']):,}")
    elif 'total_points' in data.files:
        print(f"Total points: {int(data['total_points']):,}")
    
    if 'duration' in data.files:
        print(f"Duration: {float(data['duration']):.1f} seconds")
    
    # Get timestamps
    if 'scan_timestamps' in data.files:
        timestamps = data['scan_timestamps']
    elif 'timestamps' in data.files:
        timestamps = data['timestamps']
    else:
        timestamps = np.array([])
    
    if len(timestamps) > 1:
        dt = np.diff(timestamps)
        print(f"Scan rate: {1.0/np.mean(dt):.1f} Hz (avg)")
    
    # Get height data
    if 'full_height' in data.files:
        map_height = data['full_height']
    elif 'avg_height' in data.files:
        map_height = data['avg_height']
    elif 'map_height' in data.files:
        map_height = data['map_height']
    else:
        map_height = np.array([])
    
    valid = np.isfinite(map_height)
    if np.any(valid):
        h = map_height[valid]
        print(f"\nHeight statistics (mm):")
        print(f"  Min: {h.min():.1f} mm")
        print(f"  Max: {h.max():.1f} mm")
        print(f"  Mean: {h.mean():.1f} mm")
        print(f"  Std: {h.std():.1f} mm")
    
    # Get map coordinates
    if 'full_x' in data.files:
        map_x = data['full_x'][valid] if len(map_height) > 0 else np.array([])
        map_y = data['full_y'][valid] if len(map_height) > 0 else np.array([])
    elif 'avg_x' in data.files:
        map_x = data['avg_x']
        map_y = data['avg_y']
        valid_xy = np.isfinite(data['avg_height'])
        map_x = map_x[valid_xy]
        map_y = map_y[valid_xy]
    elif 'map_x' in data.files:
        map_x = data['map_x'][valid] if len(map_height) > 0 else np.array([])
        map_y = data['map_y'][valid] if len(map_height) > 0 else np.array([])
    else:
        map_x = np.array([])
        map_y = np.array([])
    
    if len(map_x) > 0:
        print(f"\nCoverage:")
        print(f"  X: {map_x.min():.3f} to {map_x.max():.3f} m ({map_x.max()-map_x.min():.3f} m)")
        print(f"  Y: {map_y.min():.3f} to {map_y.max():.3f} m ({map_y.max()-map_y.min():.3f} m)")
        print(f"  Area: {(map_x.max()-map_x.min())*(map_y.max()-map_y.min())*10000:.1f} cm²")
    
    print("="*60)


def main():
    parser = argparse.ArgumentParser(description="View saved laser mapping data")
    parser.add_argument('file', nargs='?', help='Path to .npz file (or auto-find latest)')
    parser.add_argument('--grid', action='store_true', help='Show gridded image instead of scatter')
    parser.add_argument('--grid-res', type=float, default=0.001, help='Grid resolution in meters (default: 0.001)')
    parser.add_argument('--3d', dest='show_3d', action='store_true', help='Show 3D surface plot')
    parser.add_argument('--trajectory', action='store_true', help='Show scanning trajectory')
    parser.add_argument('--interactive', action='store_true', help='Interactive scan browser')
    parser.add_argument('--all', action='store_true', help='Show all plots')
    parser.add_argument('--cmap', default='viridis', help='Colormap (default: viridis)')
    parser.add_argument('--point-size', type=float, default=1, help='Point size for scatter plot')
    
    args = parser.parse_args()
    
    # Find file
    if args.file:
        filepath = Path(args.file)
    else:
        # Find most recent .npz file
        npz_files = list(Path('.').glob('laser_map_*.npz'))
        if not npz_files:
            print("No laser_map_*.npz files found in current directory.")
            print("Usage: python view_laser_map.py <filename.npz>")
            return
        filepath = max(npz_files, key=lambda p: p.stat().st_mtime)
        print(f"Using most recent file: {filepath}")
    
    if not filepath.exists():
        print(f"File not found: {filepath}")
        return
    
    # Load data
    data = load_map_data(filepath)
    
    # Analyze
    analyze_data(data)
    
    # Plot
    if args.all:
        args.grid = True
        args.show_3d = True
        args.trajectory = True
        args.interactive = True
    
    if args.grid:
        plot_height_map_gridded(data, grid_resolution=args.grid_res, cmap=args.cmap)
    else:
        plot_height_map(data, cmap=args.cmap, point_size=args.point_size)
    
    if args.trajectory:
        plot_scan_trajectory(data)
    
    if args.show_3d:
        plot_3d_surface(data, cmap=args.cmap)
    
    if args.interactive:
        interactive_scan_viewer(data)
    
    plt.show()


if __name__ == "__main__":
    main()
