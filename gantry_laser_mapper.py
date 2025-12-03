# gantry_laser_mapper.py
# Combined Gantry Controller + Laser Scanner for 2D Height Mapping
# Controls gantry with Xbox controller while building a live height map
# 
# The laser scans a 10cm line in X direction. As you move the gantry along Y,
# it builds up a 2D top-view map where color = height.
# Height is calculated as: actual_height = gantry_Z - laser_distance
# (gantry Z is inverse, laser gives distance from laser head)

import sys
import time
import threading
import ctypes as ct
import numpy as np
import signal
import os
from datetime import datetime
from contextlib import contextmanager
from collections import deque

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
from matplotlib.cm import ScalarMappable
import matplotlib.patches as mpatches

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

# Gantry position offsets (from your original code)
GANTRY_OFFSET_X = 0.9
GANTRY_OFFSET_Y = 1.0
GANTRY_OFFSET_Z = 1.3  # This offset + inverse gives laser height

# Mapping parameters
DOWNSAMPLE_FACTOR = 10  # Take every Nth point from laser (1280 -> 128 points per scan)
MAP_UPDATE_INTERVAL_MS = 50  # How often to update the live map
LASER_SCAN_HZ = 30  # Target laser acquisition rate

# Data saving
SAVE_RAW_DATA = True  # Save raw laser + gantry data separately
AUTO_SAVE_INTERVAL_S = 60  # Auto-save every N seconds


# ========================
# LASER SCANNER FUNCTIONS
# ========================
def check_llt_success(result: int, operation_name: str) -> bool:
    if result >= 1:
        print(f"✅ {operation_name}: SUCCESS (code: {result})")
        return True
    else:
        print(f"❌ {operation_name}: FAILED (code: {result})")
        return False


def ip_int_to_str(ip_int: int) -> str:
    return f"{ip_int & 0xFF}.{(ip_int >> 8) & 0xFF}.{(ip_int >> 16) & 0xFF}.{(ip_int >> 24) & 0xFF}"


@contextmanager
def safe_transfer(hLLT):
    try:
        ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        if ret < 1:
            raise RuntimeError(f"transfer_profiles start failed: {ret}")
        yield
    finally:
        try:
            llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
        except Exception:
            pass


def connect_scanner():
    print("=== LASER SCANNER: CREATE HANDLE ===")
    hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
    if not hLLT:
        raise RuntimeError("Failed to create LLT device handle")

    print("=== LASER SCANNER: SEARCH INTERFACES ===")
    interfaces = (ct.c_uint * 6)()
    res = llt.get_device_interfaces_fast(hLLT, interfaces, len(interfaces))
    if not check_llt_success(res, "get_device_interfaces_fast"):
        raise RuntimeError("No interfaces found")

    detected = interfaces[0]
    if detected == 0:
        raise RuntimeError("Interface slot is zero (no scanner).")

    print(f"Scanner detected at: {ip_int_to_str(detected)}")
    
    res = llt.set_device_interface(hLLT, detected, 0)
    if not check_llt_success(res, "set_device_interface"):
        raise RuntimeError(f"set_device_interface failed: {res}")

    res = llt.connect(hLLT)
    if not check_llt_success(res, "connect"):
        if res == -301:
            raise RuntimeError("Scanner in use by another software (-301).")
        elif res == -303:
            raise RuntimeError("Handle already in use (-303). Restart Python.")
        else:
            raise RuntimeError(f"Connection failed: {res}")

    print("🎉 Laser scanner connected.")
    return hLLT


def setup_scanner(hLLT):
    print("=== SCANNER SETUP ===")
    
    scanner_type = ct.c_int(0)
    res = llt.get_llt_type(hLLT, ct.byref(scanner_type))
    if not check_llt_success(res, "get_llt_type"):
        raise RuntimeError("get_llt_type failed")

    available_resolutions = (ct.c_uint * 4)()
    res = llt.get_resolutions(hLLT, available_resolutions, len(available_resolutions))
    if not check_llt_success(res, "get_resolutions"):
        raise RuntimeError("get_resolutions failed")

    res_list = [int(v) for v in available_resolutions if int(v) > 0]
    resolution = max(res_list) if res_list else int(available_resolutions[0])
    print(f"Using resolution: {resolution}")
    
    res = llt.set_resolution(hLLT, resolution)
    if not check_llt_success(res, "set_resolution"):
        raise RuntimeError("set_resolution failed")

    res = llt.set_profile_config(hLLT, llt.TProfileConfig.PROFILE)
    if not check_llt_success(res, "set_profile_config(PROFILE)"):
        raise RuntimeError("set_profile_config failed")

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
    """Get current gantry position with offsets applied"""
    try:
        pos = robot.robot_state.position_robot
        x = pos.X - GANTRY_OFFSET_X
        y = pos.Y - GANTRY_OFFSET_Y
        z = pos.Z + GANTRY_OFFSET_Z  # Z is inverse, adding offset gives actual laser height
        return x, y, z
    except Exception:
        return None, None, None


# ========================
# LASER GRABBER CLASS
# ========================
class LaserGrabber:
    """Handles laser profile acquisition"""
    
    def __init__(self, hLLT, scanner_type, resolution):
        self.hLLT = hLLT
        self.scanner_type = scanner_type
        self.resolution = int(resolution)
        
        # Buffers
        self.profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        self.lost_profiles = ct.c_int(0)
        self.x = (ct.c_double * self.resolution)()
        self.z = (ct.c_double * self.resolution)()
        self.i = (ct.c_ushort * self.resolution)()
        self.snull_us = ct.POINTER(ct.c_ushort)()
        self.snull_ui = ct.POINTER(ct.c_uint)()
        
        self.unit_scale = None
        
    def grab(self):
        """Grab one profile from laser, returns (x_laser, z_laser, intensity) or None"""
        r = llt.get_actual_profile(
            self.hLLT,
            self.profile_buffer,
            len(self.profile_buffer),
            llt.TProfileConfig.PROFILE,
            ct.byref(self.lost_profiles),
        )
        if r < 1:
            return None

        r2 = llt.convert_profile_2_values(
            self.hLLT,
            self.profile_buffer,
            self.resolution,
            llt.TProfileConfig.PROFILE,
            self.scanner_type,
            0, 1,
            self.snull_us, self.i, self.snull_us,
            self.x, self.z,
            self.snull_ui, self.snull_ui,
        )
        if r2 < 1:
            return None

        x_np = np.frombuffer(self.x, dtype=np.float64, count=self.resolution).copy()
        z_np = np.frombuffer(self.z, dtype=np.float64, count=self.resolution).copy()
        i_np = np.frombuffer(self.i, dtype=np.uint16, count=self.resolution).copy()

        # Invalidate bad data
        invalid = (i_np == 0) | (~np.isfinite(z_np)) | (z_np <= 0)
        z_np[invalid] = np.nan

        # Auto-detect unit scale (meters vs mm)
        if self.unit_scale is None:
            valid = np.isfinite(z_np)
            if np.any(valid):
                p95_z = float(np.nanpercentile(np.abs(z_np[valid]), 95))
                p95_x = float(np.nanpercentile(np.abs(x_np), 95)) if x_np.size else 0.0
                self.unit_scale = 1000.0 if (p95_z < 5.0 and p95_x < 5.0) else 1.0
                print(f"Laser unit scale: {self.unit_scale} (1=mm, 1000=m→mm)")

        if self.unit_scale and self.unit_scale != 1.0:
            x_np *= self.unit_scale
            z_np *= self.unit_scale

        return x_np, z_np, i_np


# ========================
# DATA STORAGE CLASS
# ========================
class MappingData:
    """Stores all mapping data for live display and saving"""
    
    def __init__(self, downsample=DOWNSAMPLE_FACTOR):
        self.downsample = downsample
        self.lock = threading.Lock()
        
        # Raw data storage (for saving)
        self.raw_data = []  # List of dicts with all data
        
        # Processed data for mapping (downsampled)
        self.map_x = []      # World X coordinates
        self.map_y = []      # World Y coordinates  
        self.map_height = [] # Calculated actual heights
        
        # For visualization - store as numpy arrays (more efficient)
        self.map_x_np = np.array([])
        self.map_y_np = np.array([])
        self.map_height_np = np.array([])
        
        # Statistics
        self.total_points = 0
        self.total_scans = 0
        self.start_time = time.time()
        
    def add_scan(self, laser_x, laser_z, gantry_x, gantry_y, gantry_z, timestamp=None):
        """
        Add a laser scan to the map.
        
        laser_x: X positions from laser (relative to laser center, ~+-50mm)
        laser_z: Distance from laser head in mm
        gantry_x, gantry_y, gantry_z: Gantry world position
        
        Actual height = gantry_z - laser_z (since gantry Z goes up as laser gets higher)
        World X = gantry_x + laser_x (laser scans perpendicular to gantry motion)
        World Y = gantry_y
        """
        if timestamp is None:
            timestamp = time.time()
            
        with self.lock:
            # Store raw data
            if SAVE_RAW_DATA:
                self.raw_data.append({
                    'timestamp': timestamp,
                    'gantry_x': gantry_x,
                    'gantry_y': gantry_y,
                    'gantry_z': gantry_z,
                    'laser_x': laser_x.copy(),
                    'laser_z': laser_z.copy(),
                })
            
            # Calculate world coordinates and heights
            valid = np.isfinite(laser_z)
            if not np.any(valid):
                return
                
            # Downsample for performance
            indices = np.where(valid)[0][::self.downsample]
            if len(indices) == 0:
                return
            
            lx = laser_x[indices]
            lz = laser_z[indices]
            
            # World coordinates
            # Laser X is perpendicular to gantry motion, so world_x = gantry_x + laser_x
            world_x = gantry_x + lx / 1000.0  # Convert laser mm to meters to match gantry
            world_y = np.full(len(indices), gantry_y)
            
            # Height calculation: gantry_z (laser height above ground) - laser_z (distance to surface)
            # Result is height of surface above ground
        
            actual_height = (- gantry_z) - lz  # Convert mm to meters
            print(f"Debug: gantry_z={gantry_z:.3f}, lz_sample={lz[0]:.1f}, actual_height_sample={actual_height[0]:.3f}")

            self.map_x.extend(world_x.tolist())
            self.map_y.extend(world_y.tolist())
            self.map_height.extend(actual_height.tolist())
            
            self.total_points += len(indices)
            self.total_scans += 1
            
    def get_map_arrays(self):
        """Get numpy arrays for plotting (thread-safe copy)"""
        with self.lock:
            if len(self.map_x) == 0:
                return np.array([]), np.array([]), np.array([])
            return (np.array(self.map_x), 
                    np.array(self.map_y), 
                    np.array(self.map_height))
    
    def get_center_height(self):
        """Get the most recent center point height"""
        with self.lock:
            if len(self.raw_data) == 0:
                return None
            last = self.raw_data[-1]
            lz = last['laser_z']
            gz = last['gantry_z']
            center_idx = len(lz) // 2
            if np.isfinite(lz[center_idx]):
                return gz - lz[center_idx] / 1000.0
            return None
    
    def save(self, filename=None):
        """Save all data to npz file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"laser_map_{timestamp}.npz"
        
        with self.lock:
            # Prepare raw data arrays
            if len(self.raw_data) > 0:
                timestamps = np.array([d['timestamp'] for d in self.raw_data])
                gantry_x = np.array([d['gantry_x'] for d in self.raw_data])
                gantry_y = np.array([d['gantry_y'] for d in self.raw_data])
                gantry_z = np.array([d['gantry_z'] for d in self.raw_data])
                
                # Laser data as list of arrays (variable length per scan)
                laser_x_list = [d['laser_x'] for d in self.raw_data]
                laser_z_list = [d['laser_z'] for d in self.raw_data]
                
                np.savez_compressed(
                    filename,
                    timestamps=timestamps,
                    gantry_x=gantry_x,
                    gantry_y=gantry_y,
                    gantry_z=gantry_z,
                    laser_x=np.array(laser_x_list, dtype=object),
                    laser_z=np.array(laser_z_list, dtype=object),
                    map_x=np.array(self.map_x),
                    map_y=np.array(self.map_y),
                    map_height=np.array(self.map_height),
                    total_scans=self.total_scans,
                    total_points=self.total_points,
                    duration=time.time() - self.start_time,
                )
                print(f"💾 Saved {len(self.raw_data)} scans to {filename}")
            else:
                print("No data to save.")


# ========================
# LIVE MAPPING PLOT
# ========================
class LiveMapper:
    """Live 2D height map visualization"""
    
    def __init__(self, mapping_data, show_full_map=True):
        self.data = mapping_data
        self.show_full_map = show_full_map
        
        # Setup figure with two subplots
        if show_full_map:
            self.fig, (self.ax_map, self.ax_laser) = plt.subplots(
                1, 2, figsize=(14, 6), facecolor='white'
            )
        else:
            self.fig, self.ax_laser = plt.subplots(figsize=(7, 6), facecolor='white')
            self.ax_map = None
        
        # Height map (left) - scatter plot with color = height
        if self.ax_map:
            self.ax_map.set_xlabel('X (m)')
            self.ax_map.set_ylabel('Y (m)')
            self.ax_map.set_title('Height Map (Top View)')
            self.ax_map.grid(True, alpha=0.3)
            self.ax_map.set_aspect('equal')
            
            # Initial empty scatter
            self.scatter = self.ax_map.scatter([], [], c=[], cmap='viridis', s=1, marker='s')
            
            # Colorbar
            self.cbar = self.fig.colorbar(self.scatter, ax=self.ax_map, label='Height (m)')
            
            # Current position marker
            self.pos_marker, = self.ax_map.plot([], [], 'r+', markersize=15, markeredgewidth=2)
            
        # Laser line plot (right) - real-time laser profile showing object height
        self.ax_laser.set_xlabel('Laser X (mm)')
        self.ax_laser.set_ylabel('Object Height (mm)')
        self.ax_laser.set_title('Live Object Height Profile')
        self.ax_laser.grid(True, alpha=0.3)
        self.ax_laser.set_xlim(-60, 60)
        self.ax_laser.set_ylim(-50, 200)
        self.line_laser, = self.ax_laser.plot([], [], 'g-', lw=1)
        
        # Center point marker on laser plot
        self.center_marker, = self.ax_laser.plot([], [], 'ro', markersize=8)
        
        # Text displays
        if self.ax_map:
            self.info_text = self.ax_map.text(
                0.02, 0.98, '', transform=self.ax_map.transAxes,
                verticalalignment='top', fontsize=9,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
            )
        else:
            self.info_text = self.ax_laser.text(
                0.02, 0.98, '', transform=self.ax_laser.transAxes,
                verticalalignment='top', fontsize=9,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
            )
        
        # Current laser data (updated by external thread)
        self.current_laser_x = np.array([])
        self.current_laser_z = np.array([])
        self.current_actual_height = np.array([])
        self.current_gantry_pos = (0, 0, 0)
        self.laser_lock = threading.Lock()
        
        # Map bounds
        self.x_min, self.x_max = -0.5, 0.5
        self.y_min, self.y_max = -0.5, 0.5
        self.h_min, self.h_max = 0, 0.1
        
        plt.tight_layout()
        
    def update_laser(self, laser_x, laser_z, actual_height, gantry_pos):
        """Called from laser thread to update current laser data"""
        with self.laser_lock:
            self.current_laser_x = laser_x.copy()
            self.current_laser_z = laser_z.copy()
            self.current_actual_height = actual_height.copy()
            self.current_gantry_pos = gantry_pos
            
    def init_anim(self):
        """Initialize animation"""
        artists = [self.line_laser, self.center_marker, self.info_text]
        if self.ax_map:
            artists.extend([self.scatter, self.pos_marker])
        return artists
        
    def update_anim(self, frame):
        """Animation update function"""
        artists = []
        
        # Update laser line with actual object height
        with self.laser_lock:
            lx = self.current_laser_x
            lz = self.current_laser_z
            actual_h = self.current_actual_height
            gpos = self.current_gantry_pos
            
        valid = np.isfinite(actual_h) if len(actual_h) > 0 else np.isfinite(lz)
        if np.any(valid) and len(actual_h) > 0:
            self.line_laser.set_data(lx[valid], actual_h[valid])
            
            # Center point
            center_idx = len(actual_h) // 2
            if np.isfinite(actual_h[center_idx]):
                self.center_marker.set_data([lx[center_idx]], [actual_h[center_idx]])
            
            # Auto-scale laser plot
            h_valid = actual_h[valid]
            h_min, h_max = np.nanmin(h_valid), np.nanmax(h_valid)
            margin = (h_max - h_min) * 0.1 + 10
            self.ax_laser.set_ylim(h_min - margin, h_max + margin)
            
        artists.extend([self.line_laser, self.center_marker])
        
        # Update height map
        if self.ax_map and self.show_full_map:
            map_x, map_y, map_h = self.data.get_map_arrays()
            
            if len(map_x) > 0:
                # Update scatter data
                self.scatter.set_offsets(np.c_[map_x, map_y])
                self.scatter.set_array(map_h)
                
                # Update bounds with margin
                x_margin = max(0.1, (map_x.max() - map_x.min()) * 0.1)
                y_margin = max(0.1, (map_y.max() - map_y.min()) * 0.1)
                
                new_x_min = min(self.x_min, map_x.min() - x_margin)
                new_x_max = max(self.x_max, map_x.max() + x_margin)
                new_y_min = min(self.y_min, map_y.min() - y_margin)
                new_y_max = max(self.y_max, map_y.max() + y_margin)
                
                if (new_x_min != self.x_min or new_x_max != self.x_max or
                    new_y_min != self.y_min or new_y_max != self.y_max):
                    self.x_min, self.x_max = new_x_min, new_x_max
                    self.y_min, self.y_max = new_y_min, new_y_max
                    self.ax_map.set_xlim(self.x_min, self.x_max)
                    self.ax_map.set_ylim(self.y_min, self.y_max)
                
                # Update color scale
                h_min, h_max = np.nanmin(map_h), np.nanmax(map_h)
                if h_min != h_max:
                    self.scatter.set_clim(h_min, h_max)
                    self.h_min, self.h_max = h_min, h_max
            
            # Update position marker
            gx, gy, gz = gpos
            if gx is not None:
                self.pos_marker.set_data([gx], [gy])
            
            artists.extend([self.scatter, self.pos_marker])
        
        # Update info text
        gx, gy, gz = gpos
        center_h = self.data.get_center_height()
        info = f"Gantry: X={gx:.3f} Y={gy:.3f} Z={gz:.3f}\n"
        info += f"Scans: {self.data.total_scans} | Points: {self.data.total_points}\n"
        if center_h is not None:
            info += f"Center Height: {center_h:.4f} m"
        self.info_text.set_text(info)
        artists.append(self.info_text)
        
        return artists


# ========================
# MAIN COMBINED FUNCTION
# ========================
def run_mapper(live_map=True, interval_ms=MAP_UPDATE_INTERVAL_MS):
    """
    Main function to run the combined gantry controller + laser mapper.
    
    Args:
        live_map: If True, show full height map. If False, only show laser line + center point.
        interval_ms: Plot update interval in milliseconds.
    """
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    # Initialize pygame and controller
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No Xbox controller detected. Please connect controller and try again.")
        return
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"🎮 Connected to: {joystick.get_name()}")
    
    # Shared state
    stop_flag = {"stop": False}
    speeds = {"A1": 0.0, "A2": 0.0, "A3": 0.0}
    robot_ready = {"ready": False}
    gantry_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
    
    robot = CRIController()
    hLLT = None
    active = enabled = jog_active = False
    
    # Data storage
    mapping_data = MappingData(downsample=DOWNSAMPLE_FACTOR)
    
    # Threads
    threads = []
    
    def robot_jog_loop():
        """Send jog commands to robot continuously"""
        while not stop_flag["stop"]:
            if robot_ready["ready"]:
                robot.set_jog_values(
                    A1=speeds["A1"], A2=speeds["A2"], A3=speeds["A3"],
                    A4=0.0, A5=0.0, A6=0.0,
                    E1=0.0, E2=0.0, E3=0.0,
                )
            time.sleep(JOG_DT)
    
    def laser_acquisition_loop(grabber, mapper):
        """Continuously acquire laser profiles and add to map"""
        dt = 1.0 / LASER_SCAN_HZ
        last_save = time.time()
        
        while not stop_flag["stop"]:
            t0 = time.time()
            
            # Grab laser data
            data = grabber.grab()
            if data is not None:
                laser_x, laser_z, intensity = data
                
                # Get current gantry position
                gx, gy, gz = gantry_pos["x"], gantry_pos["y"], gantry_pos["z"]
                
                # Add to mapping data
                mapping_data.add_scan(laser_x, laser_z, gx, gy, gz)
                # Calculate actual height for live display (same formula as in add_scan)
                actual_height_live = -((-gz) - laser_z)  # Object height in mm
                # Update live plotter
                mapper.update_laser(laser_x, laser_z, actual_height_live, (gx, gy, gz))
            
            # Auto-save periodically
            if SAVE_RAW_DATA and (time.time() - last_save) > AUTO_SAVE_INTERVAL_S:
                mapping_data.save()
                last_save = time.time()
            
            # Rate limiting
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    
    def controller_loop():
        """Read controller input and update speeds"""
        clock = pygame.time.Clock()
        
        while not stop_flag["stop"]:
            # Process pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    stop_flag["stop"] = True
            
            # Read controller
            raw_axis_0 = joystick.get_axis(0)
            raw_axis_1 = joystick.get_axis(1)
            raw_axis_3 = joystick.get_axis(3)
            
            axis_x = apply_deadzone(raw_axis_1)
            axis_y = apply_deadzone(-raw_axis_0)
            axis_z = apply_deadzone(raw_axis_3)
            
            speeds["A1"] = axis_y * MAX_SPEED
            speeds["A2"] = axis_x * MAX_SPEED
            speeds["A3"] = axis_z * MAX_SPEED
            
            # Emergency stop (B button)
            if joystick.get_button(1):
                speeds["A1"] = speeds["A2"] = speeds["A3"] = 0.0
                print("\n⚠️ Emergency stop!")
            
            # Exit (Start button)
            if joystick.get_button(7):
                stop_flag["stop"] = True
            
            # Save data (X button - button 2)
            if joystick.get_button(2):
                mapping_data.save()
            
            # Update gantry position
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
        
        # Reset loop
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
        
        # Connect laser scanner
        print("\n=== CONNECTING TO LASER SCANNER ===")
        hLLT = connect_scanner()
        scanner_type, resolution = setup_scanner(hLLT)
        grabber = LaserGrabber(hLLT, scanner_type, resolution)
        
        # Start transfer
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
            print("ERROR: No laser profiles received")
            return
        
        print("🎉 Laser scanner streaming!")
        
        # Create live plotter
        mapper = LiveMapper(mapping_data, show_full_map=live_map)
        
        # Print controls
        print("\n" + "="*60)
        print("GANTRY LASER MAPPER")
        print("="*60)
        print("Left Stick:      Move X/Y")
        print("Right Stick Y:   Move Z (laser height)")
        print("B Button:        Emergency stop")
        print("X Button:        Save data")
        print("Start Button:    Exit")
        print("="*60 + "\n")
        
        # Start threads
        robot_thread = threading.Thread(target=robot_jog_loop, daemon=True)
        robot_thread.start()
        threads.append(robot_thread)
        
        laser_thread = threading.Thread(target=laser_acquisition_loop, args=(grabber, mapper), daemon=True)
        laser_thread.start()
        threads.append(laser_thread)
        
        controller_thread = threading.Thread(target=controller_loop, daemon=True)
        controller_thread.start()
        threads.append(controller_thread)
        
        # Run animation (must be in main thread)
        ani = FuncAnimation(
            mapper.fig,
            mapper.update_anim,
            init_func=mapper.init_anim,
            interval=interval_ms,
            blit=False,  # blit=False for dynamic axis changes
            cache_frame_data=False,
        )
        plt.show()
        
    finally:
        print("\n=== SHUTTING DOWN ===")
        stop_flag["stop"] = True
        
        # Save data
        if mapping_data.total_scans > 0:
            mapping_data.save()
        
        # Stop laser
        try:
            llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
        except Exception:
            pass
        try:
            if hLLT:
                llt.disconnect(hLLT)
                print("Laser disconnected.")
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
        print("Exited cleanly.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Gantry Laser Mapper")
    parser.add_argument('--lite', action='store_true', 
                        help='Lite mode: only show laser line, no full map (less CPU)')
    parser.add_argument('--interval', type=int, default=MAP_UPDATE_INTERVAL_MS,
                        help='Plot update interval in ms (default: 50)')
    args = parser.parse_args()
    
    run_mapper(live_map=not args.lite, interval_ms=args.interval)
