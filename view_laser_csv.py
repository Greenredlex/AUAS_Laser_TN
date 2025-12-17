import sys
import csv
import json
import numpy as np
import scipy.interpolate
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import glob
import os

def get_latest_csv():
    list_of_files = glob.glob('scan_data_*.csv') 
    if not list_of_files:
        return None
    return max(list_of_files, key=os.path.getctime)

def view_csv(filename):
    print(f"Loading {filename}...")
    frames = []
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                frames.append(row)
    except FileNotFoundError:
        print(f"File not found: {filename}")
        return

    if not frames:
        print("No data in CSV.")
        return

    print(f"Loaded {len(frames)} frames.")

    # Check for robot data
    has_robot_data = 'robot_x' in frames[0]
    
    # Pre-process robot data for path plotting
    robot_x_arr = []
    robot_y_arr = []
    robot_z_arr = []
    
    for row in frames:
        if has_robot_data:
            robot_x_arr.append(float(row.get('robot_x', 0.0)))
            robot_y_arr.append(float(row.get('robot_y', 0.0)))
            robot_z_arr.append(float(row.get('robot_z', 0.0)))
        else:
            robot_x_arr.append(0.0)
            robot_y_arr.append(0.0)
            robot_z_arr.append(0.0)
            
    robot_x_arr = np.array(robot_x_arr)
    robot_y_arr = np.array(robot_y_arr)
    robot_z_arr = np.array(robot_z_arr)

    # Setup Plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # --- Plot 1: Laser Profile ---
    ax1.grid(True)
    ax1.set_xlabel("Laser X (mm)")
    ax1.set_ylabel("Actual Z (Laser Z + Robot Z) (mm)")
    ax1.set_title("Laser Profile")
    
    line_raw, = ax1.plot([], [], 'g.', markersize=2, label='Raw', alpha=0.3)
    line_smooth, = ax1.plot([], [], 'r-', lw=2, label='Smoothed')
    info_text = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, verticalalignment='top')
    ax1.legend(loc='upper right')

    # --- Plot 2: Robot Position ---
    ax2.grid(True)
    ax2.set_xlabel("Robot X (mm)")
    ax2.set_ylabel("Robot Y (mm)")
    ax2.set_title("Gantry Position")
    #ax2.set_aspect('equal', 'datalim')
    
    line_robot_path, = ax2.plot([], [], 'b-', lw=1, label='Path', alpha=0.5)
    point_robot_curr, = ax2.plot([], [], 'bo', markersize=8, label='Current Pos')

    ax2.legend(loc='upper right')

    # Determine global bounds for axis scaling
    all_laser_x = []
    all_actual_z = []
    
    print("Calculating bounds...")
    # Sample frames for bounds (every 10th frame to speed up if large)
    step = 1 if len(frames) < 1000 else 10
    for i in range(0, len(frames), step):
        row = frames[i]
        try:
            xv = np.array(json.loads(row['x_values']))
            zv = np.array(json.loads(row['z_values']))
            rz = robot_z_arr[i]
            
            if len(xv) > 0:
                all_laser_x.append(np.min(xv))
                all_laser_x.append(np.max(xv))
                # Apply Z offset
                actual_z = zv + rz
                all_actual_z.append(np.min(actual_z))
                all_actual_z.append(np.max(actual_z))
        except Exception:
            pass
    
    # Set limits for Laser Plot
    if all_laser_x:
        lx_min, lx_max = min(all_laser_x), max(all_laser_x)
        lz_min, lz_max = min(all_actual_z), max(all_actual_z)
        margin_x = (lx_max - lx_min) * 0.1 if lx_max != lx_min else 1.0
        margin_z = (lz_max - lz_min) * 0.1 if lz_max != lz_min else 1.0
        ax1.set_xlim(lx_min - margin_x, lx_max + margin_x)
        ax1.set_ylim(lz_min - margin_z, lz_max + margin_z)
    else:
        ax1.set_xlim(-10, 10)
        ax1.set_ylim(-10, 10)

    # Set limits for Robot Plot
    if has_robot_data and len(robot_x_arr) > 0:
        rx_min, rx_max = np.min(robot_x_arr), np.max(robot_x_arr)
        ry_min, ry_max = np.min(robot_y_arr), np.max(robot_y_arr)
        print(f"Robot X range: {rx_min} to {rx_max}")
        print(f"Robot Y range: {ry_min} to {ry_max}")
        # Add some margin
        rmargin_x = (rx_max - rx_min) * 0.1 if rx_max != rx_min else 0.5
        rmargin_y = (ry_max - ry_min) * 0.1 if ry_max != ry_min else 0.5
        print(f"Setting Robot plot limits with margin {rmargin_x}, {rmargin_y}")
        ax2.set_xlim(rx_max + rmargin_x, rx_min - rmargin_x)
        ax2.set_ylim(ry_max + rmargin_y, ry_min - rmargin_y)
    else:
        ax2.set_xlim(-1, 1)
        ax2.set_ylim(-1, 1)

    def init():
        line_raw.set_data([], [])
        line_smooth.set_data([], [])
        info_text.set_text('')
        line_robot_path.set_data([], [])
        point_robot_curr.set_data([], [])
        return line_raw, line_smooth, info_text, line_robot_path, point_robot_curr

    def update(frame_idx):
        row = frames[frame_idx]
        
        # --- Update Robot Plot ---
        # Show path up to current frame
        line_robot_path.set_data(robot_x_arr[:frame_idx+1], robot_y_arr[:frame_idx+1])
        point_robot_curr.set_data([robot_x_arr[frame_idx]], [robot_y_arr[frame_idx]])
        
        # --- Update Laser Plot ---
        try:
            xv = np.array(json.loads(row['x_values']))
            zv = np.array(json.loads(row['z_values']))
            rz = robot_z_arr[frame_idx]
            
            # Apply Z offset
            actual_z = zv + rz
            
        except Exception:
            return line_raw, line_smooth, info_text, line_robot_path, point_robot_curr
        
        # Raw data
        line_raw.set_data(xv, actual_z)

        # Smoothing
        if len(xv) > 3:
            # Sort by x for spline
            order = np.argsort(xv)
            xv_sorted = xv[order]
            zv_sorted = actual_z[order]
            
            try:
                spline = scipy.interpolate.make_smoothing_spline(xv_sorted, zv_sorted, lam=20)
                zv_spl = spline(xv_sorted)
                line_smooth.set_data(xv_sorted, zv_spl)
            except Exception:
                line_smooth.set_data([], [])
        else:
            line_smooth.set_data([], [])

        # Update text
        z_span = 0
        if len(actual_z) > 0:
            z_span = np.max(actual_z) - np.min(actual_z)
            print(np.max(actual_z))
        info_text.set_text(
            f"Frame: {row['frame_num']}\n"
            f"Robot X: {robot_x_arr[frame_idx]:.3f}\n"
            f"Robot Y: {robot_y_arr[frame_idx]:.3f}\n"
            f"Robot Z: {robot_z_arr[frame_idx]:.3f}\n"
            f"Profile Z Span: {z_span} mm"
        )
        
        return line_raw, line_smooth, info_text, line_robot_path, point_robot_curr

    ani = FuncAnimation(fig, update, frames=len(frames), init_func=init, interval=50, blit=True)
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = get_latest_csv()
    
    if filename:
        view_csv(filename)
    else:
        print("No CSV file found or provided.")
