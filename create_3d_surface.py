# create_3d_surface.py
# Reconstructs a 3D surface from laser scan profiles and robot positioning.
# Uses Open3D for high-performance visualization and processing.

import sys
import csv
import json
import numpy as np
import glob
import os

try:
    import open3d as o3d
except ImportError:
    print("Error: Open3D is not installed. Please run: pip install open3d")
    sys.exit(1)

def get_latest_csv():
    list_of_files = glob.glob('scan_data_*.csv') 
    list_of_files += glob.glob('Corrosie_meetingen/scan_data_*.csv')
    if not list_of_files:
        return None
    return max(list_of_files, key=os.path.getctime)

def load_point_cloud(filename):
    print(f"Loading {filename}...")
    points = []
    
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    # Robot position
                    rx = -float(row.get('robot_x', 0.0))
                    ry = float(row.get('robot_y', 0.0))
                    rz = float(row.get('robot_z', 0.0))
                    
                    lx_vals = np.array(json.loads(row['x_values']))
                    lz_vals = np.array(json.loads(row['z_values']))
                    
                    if len(lx_vals) == 0:
                        continue
                        
                    # Filter out invalid Z
                    valid_mask = (lz_vals > -1000) & (lz_vals < 1000)
                    lx_vals = lx_vals[valid_mask]
                    lz_vals = lz_vals[valid_mask]
                    
                    if len(lx_vals) == 0:
                        continue

                    # Transform to Global Coordinates
                    # Global X = Robot X + Laser X
                    # Global Y = Robot Y
                    # Global Z = Robot Z + Laser Z
                    
                    global_x = rx + lx_vals
                    global_y = np.full_like(global_x, ry)
                    global_z = rz + lz_vals
                    
                    # Stack into (N, 3) array
                    frame_points = np.column_stack((global_x, global_y, global_z))
                    points.append(frame_points)
                    
                except (ValueError, json.JSONDecodeError):
                    continue
                    
    except FileNotFoundError:
        print(f"File not found: {filename}")
        return None

    if not points:
        return None
        
    # Concatenate all frames
    all_points = np.vstack(points)
    print(f"Loaded {len(all_points)} points.")
    return all_points

def visualize_point_cloud(points, voxel_size=0.5):
    print("Creating Open3D PointCloud...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 1. Voxel Downsampling (Averaging)
    # This handles the "average overlapping passes" requirement efficiently
    # and reduces the point count for smoother rendering.
    print(f"Downsampling with voxel size {voxel_size}mm...")
    downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"Points after downsampling: {len(downpcd.points)}")

    # 2. Estimate Normals
    # Essential for lighting calculations to see "dents" and surface details
    print("Estimating normals...")
    downpcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )

    # 3. Color Mapping (Height-based)
    # Map Z height to color for better visualization
    print("Generating height map colors...")
    pcd_points = np.asarray(downpcd.points)
    z_values = pcd_points[:, 2]
    z_min, z_max = z_values.min(), z_values.max()
    z_range = z_max - z_min
    
    if z_range > 0:
        # Simple gradient: Blue (low) to Red (high)
        colors = np.zeros((len(pcd_points), 3))
        norm_z = (z_values - z_min) / z_range
        # Red channel
        colors[:, 0] = norm_z 
        # Blue channel
        colors[:, 2] = 1 - norm_z
        downpcd.colors = o3d.utility.Vector3dVector(colors)

    # 4. Visualization
    print("Opening Visualizer...")
    print("Controls:")
    print("  [Mouse Left] Rotate")
    print("  [Mouse Wheel] Zoom")
    print("  [Mouse Right] Pan")
    print("  [+/-] Increase/Decrease point size")
    print("  [N] Toggle normals")
    
    o3d.visualization.draw_geometries([downpcd], 
                                      window_name="3D Surface Reconstruction",
                                      width=1200, height=800,
                                      left=50, top=50)

def main():
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = get_latest_csv()
    
    if not filename:
        print("No CSV file found.")
        return

    points = load_point_cloud(filename)
    if points is None:
        print("Could not load points.")
        return
        
    # Adjust voxel_size to control the "averaging" resolution.
    # 0.2mm is high res, 1.0mm is coarser.
    visualize_point_cloud(points, voxel_size=0.2)

if __name__ == "__main__":
    main()
