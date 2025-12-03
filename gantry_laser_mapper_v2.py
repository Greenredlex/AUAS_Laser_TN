# gantry_laser_mapper_v2.py
# Combined Gantry Controller + Laser Scanner for 2D Height Mapping
# OPTIMIZED VERSION - Lower RAM usage, faster saving, full laser range display
#
# Height formula: actual_height = -((-gantry_z) - laser_z)
# This gives consistent object height regardless of laser head position

import sys
import time
import threading
import ctypes as ct
import numpy as np
import signal
from datetime import datetime
from collections import deque
import gc

# Adjust import paths
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector")
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector\libs\python_bindings")

try:
    import pygame
except ImportError:
    print("pygame not installed. Please run: pip install pygame")
    sys.exit(1)

try:
    import pyllt as llt
except ImportError:
    try:
        from pyllt import pyllt as llt
    except Exception:
        print("ERROR: Could not import pyllt.")
        sys.exit(1)

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import Normalize
import matplotlib.cm as cm

from libs.cri_lib.cri_controller import CRIController
from libs.cri_lib.robot_state import KinematicsState

# ========================
# CONFIGURATION
# ========================
IP = "192.168.3.11"
PORT = 3920

# Controller parameters
JOG_UPDATE_HZ = 50
JOG_DT = 1.0 / JOG_UPDATE_HZ
MAX_SPEED = 150.0
DEADZONE = 0.05

# Gantry position offsets
GANTRY_OFFSET_X = 0.9
GANTRY_OFFSET_Y = 1.0
GANTRY_OFFSET_Z = 1.3

# Mapping parameters - OPTIMIZED
LASER_SCAN_HZ = 30              # Target laser acquisition rate
MAP_UPDATE_INTERVAL_MS = 100    # Plot update interval (slower = less CPU)

# Live map display settings
LIVE_MAP_DOWNSAMPLE = 5         # For live display: take every Nth point from each scan
LIVE_MAP_MAX_POINTS = 50000     # Max points to keep in live display buffer (older get dropped)

# Saving settings
SAVE_FULL_RESOLUTION = True     # Save ALL laser points (for final visualization)
SAVE_AVERAGED = True            # Also save averaged/downsampled version
AVERAGE_GRID_SIZE_MM = 2.0      # Grid size for averaging (2mm = 50 points per cm)
AUTO_SAVE_INTERVAL_S = 120      # Auto-save every 2 minutes (less frequent)

# Raw data buffer - use memory-mapped file for large datasets
USE_MEMORY_EFFICIENT = True     # Use numpy memory mapping for large data
MAX_SCANS_IN_MEMORY = 3000      # Keep max this many scans in RAM, older go to temp file


# ========================
# HEIGHT CALCULATION
# ========================
def calculate_actual_height(gantry_z, laser_z):
    """
    Calculate actual object height from gantry Z and laser distance.
    
    gantry_z: Gantry Z position (with offset applied, inverse direction)
    laser_z: Distance from laser to surface in mm
    
    Returns: Object height in mm
    """
    return -((-gantry_z * 1000) - laser_z)  # Convert gantry_z from m to mm


def calculate_actual_height_m(gantry_z, laser_z_mm):
    """Same but returns height in meters"""
    return -((-gantry_z) - laser_z_mm / 1000.0)


# ========================
# LASER SCANNER FUNCTIONS
# ========================
def check_llt_success(result: int, operation_name: str) -> bool:
    if result >= 1:
        print(f"✅ {operation_name}: SUCCESS")
        return True
    else:
        print(f"❌ {operation_name}: FAILED ({result})")
        return False


def ip_int_to_str(ip_int: int) -> str:
    return f"{ip_int & 0xFF}.{(ip_int >> 8) & 0xFF}.{(ip_int >> 16) & 0xFF}.{(ip_int >> 24) & 0xFF}"


def connect_scanner():
    print("=== LASER SCANNER: CONNECTING ===")
    hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
    if not hLLT:
        raise RuntimeError("Failed to create LLT device handle")

    interfaces = (ct.c_uint * 6)()
    res = llt.get_device_interfaces_fast(hLLT, interfaces, len(interfaces))
    if res < 1:
        raise RuntimeError("No interfaces found")

    detected = interfaces[0]
    if detected == 0:
        raise RuntimeError("No scanner detected")

    print(f"Scanner at: {ip_int_to_str(detected)}")
    llt.set_device_interface(hLLT, detected, 0)
    
    res = llt.connect(hLLT)
    if res < 1:
        raise RuntimeError(f"Connection failed: {res}")

    print("🎉 Laser connected.")
    return hLLT


def setup_scanner(hLLT):
    scanner_type = ct.c_int(0)
    llt.get_llt_type(hLLT, ct.byref(scanner_type))

    available_resolutions = (ct.c_uint * 4)()
    llt.get_resolutions(hLLT, available_resolutions, len(available_resolutions))
    res_list = [int(v) for v in available_resolutions if int(v) > 0]
    resolution = max(res_list) if res_list else int(available_resolutions[0])
    
    llt.set_resolution(hLLT, resolution)
    llt.set_profile_config(hLLT, llt.TProfileConfig.PROFILE)
    
    print(f"Resolution: {resolution}")
    return scanner_type, resolution


# ========================
# HELPER FUNCTIONS
# ========================
def apply_deadzone(value, deadzone=DEADZONE):
    if abs(value) < deadzone:
        return 0.0
    sign = 1 if value > 0 else -1
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return sign * scaled


def get_gantry_position(robot):
    try:
        pos = robot.robot_state.position_robot
        x = pos.X - GANTRY_OFFSET_X
        y = pos.Y - GANTRY_OFFSET_Y
        z = pos.Z + GANTRY_OFFSET_Z
        return x, y, z
    except Exception:
        return None, None, None


# ========================
# LASER GRABBER CLASS
# ========================
class LaserGrabber:
    def __init__(self, hLLT, scanner_type, resolution):
        self.hLLT = hLLT
        self.scanner_type = scanner_type
        self.resolution = int(resolution)
        
        self.profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        self.lost_profiles = ct.c_int(0)
        self.x = (ct.c_double * self.resolution)()
        self.z = (ct.c_double * self.resolution)()
        self.i = (ct.c_ushort * self.resolution)()
        self.snull_us = ct.POINTER(ct.c_ushort)()
        self.snull_ui = ct.POINTER(ct.c_uint)()
        self.unit_scale = None
        
    def grab(self):
        r = llt.get_actual_profile(
            self.hLLT, self.profile_buffer, len(self.profile_buffer),
            llt.TProfileConfig.PROFILE, ct.byref(self.lost_profiles),
        )
        if r < 1:
            return None

        r2 = llt.convert_profile_2_values(
            self.hLLT, self.profile_buffer, self.resolution,
            llt.TProfileConfig.PROFILE, self.scanner_type, 0, 1,
            self.snull_us, self.i, self.snull_us,
            self.x, self.z, self.snull_ui, self.snull_ui,
        )
        if r2 < 1:
            return None

        x_np = np.frombuffer(self.x, dtype=np.float64, count=self.resolution).copy()
        z_np = np.frombuffer(self.z, dtype=np.float64, count=self.resolution).copy()
        i_np = np.frombuffer(self.i, dtype=np.uint16, count=self.resolution).copy()

        invalid = (i_np == 0) | (~np.isfinite(z_np)) | (z_np <= 0)
        z_np[invalid] = np.nan

        if self.unit_scale is None:
            valid = np.isfinite(z_np)
            if np.any(valid):
                p95_z = float(np.nanpercentile(np.abs(z_np[valid]), 95))
                p95_x = float(np.nanpercentile(np.abs(x_np), 95))
                self.unit_scale = 1000.0 if (p95_z < 5.0 and p95_x < 5.0) else 1.0
                print(f"Laser unit scale: {self.unit_scale}")

        if self.unit_scale and self.unit_scale != 1.0:
            x_np *= self.unit_scale
            z_np *= self.unit_scale

        return x_np, z_np, i_np


# ========================
# EFFICIENT DATA STORAGE
# ========================
class EfficientDataStorage:
    """
    Memory-efficient storage for laser mapping data.
    - Keeps limited data in RAM for live display
    - Streams full data to disk incrementally
    - Supports averaging/downsampling on save
    """
    
    def __init__(self, output_prefix="laser_scan"):
        self.lock = threading.Lock()
        self.output_prefix = output_prefix
        self.timestamp_start = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Live display buffer (limited size, circular)
        self.live_x = deque(maxlen=LIVE_MAP_MAX_POINTS)
        self.live_y = deque(maxlen=LIVE_MAP_MAX_POINTS)
        self.live_h = deque(maxlen=LIVE_MAP_MAX_POINTS)
        
        # Full resolution data - write to temp file incrementally
        self.temp_filename = f"_temp_{self.timestamp_start}.bin"
        self.temp_file = None
        self.scan_count = 0
        self.point_count = 0
        
        # Metadata for each scan (small, kept in memory)
        self.scan_metadata = []  # List of (timestamp, gantry_x, gantry_y, gantry_z, n_points)
        
        # For averaging - accumulate in grid
        self.grid_data = {}  # (grid_x, grid_y) -> list of heights
        
        self.start_time = time.time()
        self._init_temp_file()
        
    def _init_temp_file(self):
        """Initialize temporary binary file for streaming writes"""
        try:
            self.temp_file = open(self.temp_filename, 'wb')
        except Exception as e:
            print(f"Warning: Could not create temp file: {e}")
            self.temp_file = None
    
    def add_scan(self, laser_x, laser_z, gantry_x, gantry_y, gantry_z, timestamp=None):
        """
        Add a laser scan. Efficiently stores data.
        
        laser_x, laser_z: Full laser profile in mm
        gantry_x, gantry_y, gantry_z: Gantry position in meters
        """
        if timestamp is None:
            timestamp = time.time()
        
        # Calculate actual heights
        valid = np.isfinite(laser_z)
        if not np.any(valid):
            return
        
        valid_indices = np.where(valid)[0]
        lx = laser_x[valid_indices]
        lz = laser_z[valid_indices]
        
        # World coordinates (meters)
        world_x = gantry_x + lx / 1000.0
        world_y = np.full(len(valid_indices), gantry_y)
        
        # Actual height using consistent formula
        actual_height = calculate_actual_height(gantry_z, lz)  # Returns mm
        
        with self.lock:
            n_points = len(valid_indices)
            
            # 1. Add downsampled data to live display buffer
            downsample_indices = np.arange(0, n_points, LIVE_MAP_DOWNSAMPLE)
            for idx in downsample_indices:
                self.live_x.append(world_x[idx])
                self.live_y.append(world_y[idx])
                self.live_h.append(actual_height[idx])
            
            # 2. Write full resolution to temp file (binary, fast)
            if self.temp_file and SAVE_FULL_RESOLUTION:
                # Pack: timestamp(f8), gx(f4), gy(f4), gz(f4), n_points(i4), then x,z pairs (f4,f4)
                header = np.array([timestamp, gantry_x, gantry_y, gantry_z, n_points], 
                                  dtype=np.float32)
                self.temp_file.write(header.tobytes())
                
                # Write laser data compactly
                laser_data = np.column_stack([lx.astype(np.float32), lz.astype(np.float32)])
                self.temp_file.write(laser_data.tobytes())
            
            # 3. Add to grid for averaging
            if SAVE_AVERAGED:
                grid_size = AVERAGE_GRID_SIZE_MM / 1000.0  # Convert to meters
                for i in range(n_points):
                    gx = int(world_x[i] / grid_size)
                    gy = int(world_y[i] / grid_size)
                    key = (gx, gy)
                    if key not in self.grid_data:
                        self.grid_data[key] = []
                    self.grid_data[key].append(actual_height[i])
            
            # 4. Store metadata
            self.scan_metadata.append((timestamp, gantry_x, gantry_y, gantry_z, n_points))
            self.scan_count += 1
            self.point_count += n_points
    
    def get_live_data(self):
        """Get data for live display (thread-safe, returns copies)"""
        with self.lock:
            if len(self.live_x) == 0:
                return np.array([]), np.array([]), np.array([])
            return (np.array(self.live_x), 
                    np.array(self.live_y), 
                    np.array(self.live_h))
    
    def get_averaged_data(self):
        """Get averaged grid data"""
        with self.lock:
            if not self.grid_data:
                return np.array([]), np.array([]), np.array([])
            
            grid_size = AVERAGE_GRID_SIZE_MM / 1000.0
            x_list, y_list, h_list = [], [], []
            
            for (gx, gy), heights in self.grid_data.items():
                x_list.append(gx * grid_size + grid_size / 2)
                y_list.append(gy * grid_size + grid_size / 2)
                h_list.append(np.nanmean(heights))
            
            return np.array(x_list), np.array(y_list), np.array(h_list)
    
    def save(self, filename=None):
        """Save all data to NPZ file"""
        if filename is None:
            filename = f"laser_map_{self.timestamp_start}.npz"
        
        print(f"Saving {self.scan_count} scans ({self.point_count:,} points)...")
        t0 = time.time()
        
        with self.lock:
            # Close temp file for reading
            if self.temp_file:
                self.temp_file.flush()
                self.temp_file.close()
                self.temp_file = None
            
            save_dict = {
                'scan_count': self.scan_count,
                'point_count': self.point_count,
                'duration': time.time() - self.start_time,
            }
            
            # Save metadata
            if self.scan_metadata:
                meta = np.array(self.scan_metadata, dtype=np.float64)
                save_dict['scan_timestamps'] = meta[:, 0]
                save_dict['scan_gantry_x'] = meta[:, 1]
                save_dict['scan_gantry_y'] = meta[:, 2]
                save_dict['scan_gantry_z'] = meta[:, 3]
                save_dict['scan_n_points'] = meta[:, 4].astype(np.int32)
            
            # Save averaged data (small, fast)
            if SAVE_AVERAGED and self.grid_data:
                avg_x, avg_y, avg_h = self.get_averaged_data()
                save_dict['avg_x'] = avg_x.astype(np.float32)
                save_dict['avg_y'] = avg_y.astype(np.float32)
                save_dict['avg_height'] = avg_h.astype(np.float32)
            
            # Read and save full resolution data from temp file
            if SAVE_FULL_RESOLUTION and os.path.exists(self.temp_filename):
                try:
                    all_world_x = []
                    all_world_y = []
                    all_height = []
                    
                    with open(self.temp_filename, 'rb') as f:
                        while True:
                            header_bytes = f.read(5 * 4)  # 5 float32s
                            if len(header_bytes) < 5 * 4:
                                break
                            
                            header = np.frombuffer(header_bytes, dtype=np.float32)
                            ts, gx, gy, gz, n = header
                            n = int(n)
                            
                            data_bytes = f.read(n * 2 * 4)  # n pairs of float32
                            if len(data_bytes) < n * 2 * 4:
                                break
                            
                            data = np.frombuffer(data_bytes, dtype=np.float32).reshape(-1, 2)
                            lx, lz = data[:, 0], data[:, 1]
                            
                            world_x = gx + lx / 1000.0
                            world_y = np.full(n, gy)
                            height = calculate_actual_height(gz, lz)
                            
                            all_world_x.append(world_x)
                            all_world_y.append(world_y)
                            all_height.append(height)
                    
                    if all_world_x:
                        save_dict['full_x'] = np.concatenate(all_world_x).astype(np.float32)
                        save_dict['full_y'] = np.concatenate(all_world_y).astype(np.float32)
                        save_dict['full_height'] = np.concatenate(all_height).astype(np.float32)
                
                except Exception as e:
                    print(f"Warning: Error reading temp file: {e}")
            
            # Save
            np.savez_compressed(filename, **save_dict)
            
            # Reopen temp file for continued writing
            self._init_temp_file()
        
        elapsed = time.time() - t0
        print(f"💾 Saved to {filename} in {elapsed:.2f}s")
        
        # Cleanup temp file on final save
        # (Keep it during session for incremental saves)
    
    def cleanup(self):
        """Clean up temp files"""
        with self.lock:
            if self.temp_file:
                self.temp_file.close()
                self.temp_file = None
            
            try:
                import os
                if os.path.exists(self.temp_filename):
                    os.remove(self.temp_filename)
            except Exception:
                pass


# ========================
# LIVE PLOTTER (OPTIMIZED)
# ========================
class LiveMapperV2:
    """
    Optimized live visualization with 3 panels:
    1. Height map (top view) - shows full 10cm laser range
    2. Laser profile (like contiuouslasertest.py) - X vs Z distance
    3. Object height profile - X vs actual height
    """
    
    def __init__(self, data_storage):
        self.data = data_storage
        
        # Create figure with 3 subplots
        self.fig = plt.figure(figsize=(16, 6), facecolor='white')
        
        # Left: Height map (top view)
        self.ax_map = self.fig.add_subplot(131)
        self.ax_map.set_xlabel('X (m)')
        self.ax_map.set_ylabel('Y (m)')
        self.ax_map.set_title('Height Map (Top View)')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_aspect('equal')
        
        # Use imshow for efficiency instead of scatter
        # We'll update with a rasterized image
        self.map_resolution = 0.002  # 2mm per pixel
        self.map_img = None
        self.map_extent = [-0.2, 0.2, -0.2, 0.2]  # Will be updated
        
        # Position marker
        self.pos_marker, = self.ax_map.plot([], [], 'r+', markersize=15, markeredgewidth=2)
        
        # Colorbar placeholder
        self.cbar = None
        
        # Middle: Laser distance profile (like contiuouslasertest.py)
        self.ax_laser = self.fig.add_subplot(132)
        self.ax_laser.set_xlabel('Laser X (mm)')
        self.ax_laser.set_ylabel('Distance from laser (mm)')
        self.ax_laser.set_title('Laser Profile (Range Check)')
        self.ax_laser.grid(True, alpha=0.3)
        self.ax_laser.set_xlim(-60, 60)
        self.ax_laser.set_ylim(0, 300)
        self.line_laser, = self.ax_laser.plot([], [], 'g-', lw=1.5)
        
        # Right: Object height profile
        self.ax_height = self.fig.add_subplot(133)
        self.ax_height.set_xlabel('Laser X (mm)')
        self.ax_height.set_ylabel('Object Height (mm)')
        self.ax_height.set_title('Object Height Profile')
        self.ax_height.grid(True, alpha=0.3)
        self.ax_height.set_xlim(-60, 60)
        self.ax_height.set_ylim(-50, 200)
        self.line_height, = self.ax_height.plot([], [], 'b-', lw=1.5)
        self.center_marker, = self.ax_height.plot([], [], 'ro', markersize=8)
        
        # Info text
        self.info_text = self.fig.text(
            0.02, 0.02, '', fontsize=9,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )
        
        # Current laser data
        self.current_laser_x = np.array([])
        self.current_laser_z = np.array([])
        self.current_height = np.array([])
        self.current_gantry_pos = (0, 0, 0)
        self.laser_lock = threading.Lock()
        
        # Map data cache
        self.last_map_update = 0
        self.map_update_interval = 0.5  # Update map image every 0.5s
        
        plt.tight_layout()
        
    def update_laser(self, laser_x, laser_z, actual_height, gantry_pos):
        """Called from laser thread"""
        with self.laser_lock:
            self.current_laser_x = laser_x.copy()
            self.current_laser_z = laser_z.copy()
            self.current_height = actual_height.copy()
            self.current_gantry_pos = gantry_pos
    
    def init_anim(self):
        return [self.line_laser, self.line_height, self.center_marker, 
                self.pos_marker, self.info_text]
    
    def update_anim(self, frame):
        with self.laser_lock:
            lx = self.current_laser_x
            lz = self.current_laser_z
            lh = self.current_height
            gpos = self.current_gantry_pos
        
        gx, gy, gz = gpos
        
        # Update laser distance profile
        valid = np.isfinite(lz)
        if np.any(valid):
            self.line_laser.set_data(lx[valid], lz[valid])
            
            # Auto-scale Y for laser plot
            z_valid = lz[valid]
            z_min, z_max = np.nanmin(z_valid), np.nanmax(z_valid)
            margin = max(20, (z_max - z_min) * 0.1)
            self.ax_laser.set_ylim(max(0, z_min - margin), z_max + margin)
        
        # Update height profile
        valid_h = np.isfinite(lh) if len(lh) > 0 else valid
        if np.any(valid_h) and len(lh) > 0:
            self.line_height.set_data(lx[valid_h], lh[valid_h])
            
            # Center marker
            center_idx = len(lh) // 2
            if center_idx < len(lh) and np.isfinite(lh[center_idx]):
                self.center_marker.set_data([lx[center_idx]], [lh[center_idx]])
            
            # Auto-scale Y for height plot
            h_valid = lh[valid_h]
            h_min, h_max = np.nanmin(h_valid), np.nanmax(h_valid)
            margin = max(20, (h_max - h_min) * 0.1)
            self.ax_height.set_ylim(h_min - margin, h_max + margin)
        
        # Update position marker on map
        if gx is not None:
            self.pos_marker.set_data([gx], [gy])
        
        # Update map image periodically (expensive operation)
        now = time.time()
        if now - self.last_map_update > self.map_update_interval:
            self._update_map_image()
            self.last_map_update = now
        
        # Update info text
        center_h = lh[len(lh)//2] if len(lh) > 0 and np.isfinite(lh[len(lh)//2]) else None
        info = f"Gantry: X={gx:.3f} Y={gy:.3f} Z={gz:.3f}\n"
        info += f"Scans: {self.data.scan_count} | Points: {self.data.point_count:,}\n"
        if center_h is not None:
            info += f"Center Height: {center_h:.1f} mm"
        self.info_text.set_text(info)
        
        return [self.line_laser, self.line_height, self.center_marker,
                self.pos_marker, self.info_text]
    
    def _update_map_image(self):
        """Rasterize live data into an image for efficient display"""
        map_x, map_y, map_h = self.data.get_live_data()
        
        if len(map_x) == 0:
            return
        
        # Determine bounds
        x_min, x_max = np.min(map_x), np.max(map_x)
        y_min, y_max = np.min(map_y), np.max(map_y)
        
        # Add margin
        x_margin = max(0.05, (x_max - x_min) * 0.1)
        y_margin = max(0.05, (y_max - y_min) * 0.1)
        x_min -= x_margin
        x_max += x_margin
        y_min -= y_margin
        y_max += y_margin
        
        # Create grid
        nx = max(10, int((x_max - x_min) / self.map_resolution))
        ny = max(10, int((y_max - y_min) / self.map_resolution))
        
        # Limit grid size for performance
        max_grid = 500
        if nx > max_grid:
            nx = max_grid
        if ny > max_grid:
            ny = max_grid
        
        # Bin data into grid
        grid = np.full((ny, nx), np.nan, dtype=np.float32)
        counts = np.zeros((ny, nx), dtype=np.int32)
        
        # Convert to grid indices
        xi = ((map_x - x_min) / (x_max - x_min) * (nx - 1)).astype(np.int32)
        yi = ((map_y - y_min) / (y_max - y_min) * (ny - 1)).astype(np.int32)
        
        # Clip to valid range
        xi = np.clip(xi, 0, nx - 1)
        yi = np.clip(yi, 0, ny - 1)
        
        # Accumulate (use numpy for speed)
        for i in range(len(map_x)):
            if np.isfinite(map_h[i]):
                if np.isnan(grid[yi[i], xi[i]]):
                    grid[yi[i], xi[i]] = map_h[i]
                    counts[yi[i], xi[i]] = 1
                else:
                    grid[yi[i], xi[i]] += map_h[i]
                    counts[yi[i], xi[i]] += 1
        
        # Average
        mask = counts > 0
        grid[mask] /= counts[mask]
        
        # Update or create image
        extent = [x_min, x_max, y_min, y_max]
        
        if self.map_img is None:
            self.map_img = self.ax_map.imshow(
                grid, origin='lower', extent=extent, aspect='auto',
                cmap='viridis', interpolation='nearest'
            )
            self.cbar = self.fig.colorbar(self.map_img, ax=self.ax_map, label='Height (mm)')
        else:
            self.map_img.set_data(grid)
            self.map_img.set_extent(extent)
            
            # Update color limits
            valid_data = grid[~np.isnan(grid)]
            if len(valid_data) > 0:
                vmin, vmax = np.percentile(valid_data, [2, 98])
                self.map_img.set_clim(vmin, vmax)
        
        self.ax_map.set_xlim(x_min, x_max)
        self.ax_map.set_ylim(y_min, y_max)


# ========================
# MAIN FUNCTION
# ========================
import os

def run_mapper_v2(interval_ms=MAP_UPDATE_INTERVAL_MS):
    """Run the optimized gantry laser mapper"""
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    # Initialize pygame
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No Xbox controller detected.")
        return
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"🎮 Controller: {joystick.get_name()}")
    
    # Shared state
    stop_flag = {"stop": False}
    speeds = {"A1": 0.0, "A2": 0.0, "A3": 0.0}
    robot_ready = {"ready": False}
    gantry_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
    
    robot = CRIController()
    hLLT = None
    active = enabled = jog_active = False
    
    # Efficient data storage
    data_storage = EfficientDataStorage()
    
    def robot_jog_loop():
        while not stop_flag["stop"]:
            if robot_ready["ready"]:
                robot.set_jog_values(
                    A1=speeds["A1"], A2=speeds["A2"], A3=speeds["A3"],
                    A4=0.0, A5=0.0, A6=0.0,
                    E1=0.0, E2=0.0, E3=0.0,
                )
            time.sleep(JOG_DT)
    
    def laser_acquisition_loop(grabber, plotter):
        dt = 1.0 / LASER_SCAN_HZ
        last_save = time.time()
        
        while not stop_flag["stop"]:
            t0 = time.time()
            
            data = grabber.grab()
            if data is not None:
                laser_x, laser_z, _ = data
                gx, gy, gz = gantry_pos["x"], gantry_pos["y"], gantry_pos["z"]
                
                # Add to storage
                data_storage.add_scan(laser_x, laser_z, gx, gy, gz)
                
                # Calculate height for live display
                actual_height = calculate_actual_height(gz, laser_z)
                
                # Update plotter
                plotter.update_laser(laser_x, laser_z, actual_height, (gx, gy, gz))
            
            # Auto-save (less frequent)
            if (time.time() - last_save) > AUTO_SAVE_INTERVAL_S:
                data_storage.save()
                last_save = time.time()
                gc.collect()  # Help with memory
            
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    
    def controller_loop():
        clock = pygame.time.Clock()
        
        while not stop_flag["stop"]:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    stop_flag["stop"] = True
            
            raw_0 = joystick.get_axis(0)
            raw_1 = joystick.get_axis(1)
            raw_3 = joystick.get_axis(3)
            
            speeds["A1"] = apply_deadzone(-raw_0) * MAX_SPEED
            speeds["A2"] = apply_deadzone(raw_1) * MAX_SPEED
            speeds["A3"] = apply_deadzone(raw_3) * MAX_SPEED
            
            if joystick.get_button(1):  # B
                speeds["A1"] = speeds["A2"] = speeds["A3"] = 0.0
                print("\n⚠️ Emergency stop!")
            
            if joystick.get_button(7):  # Start
                stop_flag["stop"] = True
            
            if joystick.get_button(2):  # X - manual save
                data_storage.save()
            
            gx, gy, gz = get_gantry_position(robot)
            if gx is not None:
                gantry_pos["x"] = gx
                gantry_pos["y"] = gy
                gantry_pos["z"] = gz
            
            clock.tick(JOG_UPDATE_HZ)
    
    try:
        # Connect robot
        print("=== CONNECTING TO GANTRY ===")
        if not robot.connect(IP, PORT):
            print("Robot connect failed")
            return
        
        try:
            robot.wait_for_status_update(timeout=2.0)
        except Exception:
            pass
        
        active = robot.set_active_control(True)
        
        ready_deadline = time.time() + 10.0
        while time.time() < ready_deadline:
            robot.wait_for_status_update(timeout=1.0)
            with robot.robot_state_lock:
                rs = robot.robot_state
                kin_ok = (rs.kinematics_state == KinematicsState.NO_ERROR and
                          rs.combined_axes_error == "NoError")
            if kin_ok:
                break
            robot.reset()
        
        enabled = robot.enable()
        if not enabled:
            print("Robot enable failed")
            return
        
        robot.start_jog()
        jog_active = True
        robot_ready["ready"] = True
        print("🎉 Gantry ready!")
        
        # Connect laser
        print("\n=== CONNECTING TO LASER ===")
        hLLT = connect_scanner()
        scanner_type, resolution = setup_scanner(hLLT)
        grabber = LaserGrabber(hLLT, scanner_type, resolution)
        
        ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        if ret < 1:
            raise RuntimeError(f"transfer_profiles failed: {ret}")
        
        # Wait for first profile
        t0 = time.time()
        while time.time() - t0 < 5.0:
            if grabber.grab() is not None:
                break
            time.sleep(0.01)
        else:
            print("ERROR: No laser profiles")
            return
        
        print("🎉 Laser streaming!")
        
        # Create plotter
        plotter = LiveMapperV2(data_storage)
        
        print("\n" + "="*60)
        print("GANTRY LASER MAPPER V2 (Optimized)")
        print("="*60)
        print("Left Stick:      Move X/Y")
        print("Right Stick Y:   Move Z (laser height)")
        print("B Button:        Emergency stop")
        print("X Button:        Save data")
        print("Start Button:    Exit")
        print("="*60 + "\n")
        
        # Start threads
        threading.Thread(target=robot_jog_loop, daemon=True).start()
        threading.Thread(target=laser_acquisition_loop, args=(grabber, plotter), daemon=True).start()
        threading.Thread(target=controller_loop, daemon=True).start()
        
        # Run animation
        ani = FuncAnimation(
            plotter.fig,
            plotter.update_anim,
            init_func=plotter.init_anim,
            interval=interval_ms,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()
        
    finally:
        print("\n=== SHUTTING DOWN ===")
        stop_flag["stop"] = True
        
        # Final save
        if data_storage.scan_count > 0:
            data_storage.save()
        data_storage.cleanup()
        
        # Stop laser
        try:
            llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
        except Exception:
            pass
        try:
            if hLLT:
                llt.disconnect(hLLT)
        except Exception:
            pass
        
        # Stop robot
        try:
            if jog_active:
                robot.stop_jog()
        except Exception:
            pass
        try:
            robot.stop_move()
        except Exception:
            pass
        if enabled:
            try:
                robot.disable()
            except Exception:
                pass
        if active:
            try:
                robot.set_active_control(False)
            except Exception:
                pass
        try:
            robot.close()
        except Exception:
            pass
        
        pygame.joystick.quit()
        pygame.quit()
        gc.collect()
        print("Exited cleanly.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Gantry Laser Mapper V2")
    parser.add_argument('--interval', type=int, default=MAP_UPDATE_INTERVAL_MS,
                        help='Plot update interval in ms (default: 100)')
    parser.add_argument('--no-full-save', action='store_true',
                        help='Disable saving full resolution data (only averaged)')
    args = parser.parse_args()
    
    if args.no_full_save:
        SAVE_FULL_RESOLUTION = False
    
    run_mapper_v2(interval_ms=args.interval)
