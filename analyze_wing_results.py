import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import json
import glob
import os
from scipy.interpolate import interp1d

# --- CONFIGURATION ---
NACA_CODE = '2412'
CHORD_LENGTH = 210  # mm (approximate based on PDF scaling)
CSV_PATTERN = os.path.join("wing_meetingen", "meting*.csv")  # Pattern to find your files

def naca4_symmetric(code, chord, n_points=200):
    """Generates theoretical NACA 4-digit airfoil coordinates."""
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:]) / 100.0
    
    x = np.linspace(0, chord, n_points)
    yt = 5 * t * chord * (0.2969 * np.sqrt(x/chord) - 0.1260 * (x/chord) - 
                          0.3516 * (x/chord)**2 + 0.2843 * (x/chord)**3 - 
                          0.1015 * (x/chord)**4)
    
    # Camber line calculation
    yc = np.zeros_like(x)
    for i, xi in enumerate(x):
        if xi < p * chord:
            yc[i] = m / p**2 * (2 * p * (xi/chord) - (xi/chord)**2)
        else:
            yc[i] = m / (1 - p)**2 * ((1 - 2 * p) + 2 * p * (xi/chord) - (xi/chord)**2)
            
    # For this thesis we mostly care about the Upper Surface
    x_upper = x
    y_upper = (yc + yt) * chord # Scale check
    # Note: simplified upper surface calculation for plotting context
    return x_upper, yt 

def process_csv(filename):
    print(f"Processing {filename}...")
    points = []
    try:
        df = pd.read_csv(filename)
        for _, row in df.iterrows():
            try:
                rx, rz = -float(row['robot_x']), float(row['robot_z'])
                lx_vals = np.array(json.loads(row['x_values']))
                lz_vals = np.array(json.loads(row['z_values']))
                
                # Transform to Global (assuming linear pass)
                # Global Z = Robot Z + Laser Z
                global_z = rz + lz_vals
                # Global X = Robot X + Laser X (simplified alignment)
                global_x = rx + lx_vals
                
                # Filter noise (Wing is likely within specific Z range)
                # Adjust these thresholds based on your specific setup
                mask = (global_z > -300) & (global_z < 300) 
                
                points.append(np.column_stack((global_x[mask], global_z[mask])))
            except Exception:
                continue
    except Exception as e:
        print(f"Error reading {filename}: {e}")
        return None
        
    if not points: return None
    return np.vstack(points)

# --- MAIN EXECUTION ---
files = sorted(glob.glob(os.path.join("wing_meetingen", "meting*.csv")))
if not files:
    print("No CSV files found.")
    exit()

plt.figure(figsize=(12, 8))

statistics = []

for fname in files:
    # SKIP METING 2 if it's too messy, or process it:
    # if "meting2" in fname: continue 
    
    data = process_csv(fname)
    if data is None: continue

    # 1. Sort by X to allow interpolation
    # Filter specific section to analyze reproducibility (e.g. middle of wing)
    # Removing leading/trailing edge noise for stat calculation
    clean_mask = (data[:, 0] > -50) & (data[:, 0] < 50) 
    clean_data = data[clean_mask]
    
    if len(clean_data) == 0: continue

    # Calculate local smoothness/noise (Standard Deviation from a smooth curve)
    # We fit a spline and measure distance of points from spline
    x_s = clean_data[:, 0]
    z_s = clean_data[:, 1]
    
    # Sort for spline
    sort_idx = np.argsort(x_s)
    x_s = x_s[sort_idx]
    z_s = z_s[sort_idx]
    
    # Simple smoothing to find "surface" vs "noise"
    # Sigma 3 to remove heavy outliers before stat calc
    try:
        # Fit a polynomial to represent the "ideal" surface of this specific scan
        z_smooth = np.poly1d(np.polyfit(x_s, z_s, 5))(x_s)
        
        # Calculate residuals (Noise + Texture)
        residuals = np.abs(z_s - z_smooth)
        avg_deviation = np.mean(residuals)
        std_deviation = np.std(residuals)
        
        statistics.append({
            "File": fname,
            "Points": len(x_s),
            "Mean_Error_mm": avg_deviation,
            "Std_Dev_mm": std_deviation
        })
        
        # Normalize Z for plotting overlay (center them)
        z_norm = z_s - np.mean(z_s)
        plt.scatter(x_s, z_norm, s=0.1, label=f"{fname} (Std: {std_deviation:.3f}mm)")
        
    except Exception as e:
        print(f"Stat calc failed for {fname}: {e}")

# Generate Theoretical NACA for visual comparison
tx, ty = naca4_symmetric(NACA_CODE, CHORD_LENGTH)
# Align theoretical to 0 mean for visual comparison
plt.plot(tx - (CHORD_LENGTH/2), ty - np.mean(ty), 'k--', linewidth=2, label=f"Theoretical NACA {NACA_CODE}")

plt.title("Reproducibility Analysis: Overlay of 4 Scan Profiles")
plt.xlabel("Position X (mm)")
plt.ylabel("Normalized Height Z (mm)")
plt.legend()
plt.grid(True, which='both', linestyle='--')
plt.tight_layout()
plt.savefig("reproducibility_graph.png")
plt.show()

# Print Stats for Thesis Table
print("\n=== STATISTICS FOR THESIS ===")
stat_df = pd.DataFrame(statistics)
print(stat_df)
print("\nAverage Reproducibility (Std Dev):", stat_df['Std_Dev_mm'].mean())