# gantry_laser_mapper_lite.py
# Lightweight version - saves all data but minimal live visualization
# Shows: laser line profile + center point height indicator
# Does NOT compute/render the full 2D height map live (less CPU intensive)

import sys
import time
import threading
import ctypes as ct
import numpy as np
import signal
from datetime import datetime
from contextlib import contextmanager

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
from matplotlib.ticker import ScalarFormatter

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

# Data parameters
LASER_SCAN_HZ = 30
AUTO_SAVE_INTERVAL_S = 30


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

        if self.unit_scale and self.unit_scale != 1.0:
            x_np *= self.unit_scale
            z_np *= self.unit_scale

        return x_np, z_np, i_np


# ========================
# DATA STORAGE CLASS (LITE)
# ========================
class DataRecorder:
    """Records all data for later visualization"""
    
    def __init__(self):
        self.lock = threading.Lock()
        self.raw_data = []
        self.total_scans = 0
        self.start_time = time.time()
        
    def add_scan(self, laser_x, laser_z, gantry_x, gantry_y, gantry_z, timestamp=None):
        if timestamp is None:
            timestamp = time.time()
            
        with self.lock:
            self.raw_data.append({
                'timestamp': timestamp,
                'gantry_x': gantry_x,
                'gantry_y': gantry_y,
                'gantry_z': gantry_z,
                'laser_x': laser_x.copy(),
                'laser_z': laser_z.copy(),
            })
            self.total_scans += 1
    
    def get_last_center_height(self):
        """Get center point height from last scan"""
        with self.lock:
            if len(self.raw_data) == 0:
                return None, None, None
            last = self.raw_data[-1]
            lz = last['laser_z']
            gz = last['gantry_z']
            center_idx = len(lz) // 2
            if np.isfinite(lz[center_idx]):
                actual_height = calculate_actual_height(gz, lz[center_idx])
                return actual_height, lz[center_idx], gz
            return None, None, gz
    
    def save(self, filename=None):
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"laser_map_{timestamp}.npz"
        
        with self.lock:
            if len(self.raw_data) == 0:
                print("No data to save.")
                return
                
            timestamps = np.array([d['timestamp'] for d in self.raw_data])
            gantry_x = np.array([d['gantry_x'] for d in self.raw_data])
            gantry_y = np.array([d['gantry_y'] for d in self.raw_data])
            gantry_z = np.array([d['gantry_z'] for d in self.raw_data])
            laser_x_list = [d['laser_x'] for d in self.raw_data]
            laser_z_list = [d['laser_z'] for d in self.raw_data]
            
            # Also compute the downsampled map for visualization
            downsample = 10
            map_x, map_y, map_height = [], [], []
            
            for d in self.raw_data:
                lx = d['laser_x']
                lz = d['laser_z']
                gx, gy, gz = d['gantry_x'], d['gantry_y'], d['gantry_z']
                
                valid = np.isfinite(lz)
                if not np.any(valid):
                    continue
                    
                indices = np.where(valid)[0][::downsample]
                if len(indices) == 0:
                    continue
                
                world_x = gx + lx[indices] / 1000.0
                world_y = np.full(len(indices), gy)
                actual_height = calculate_actual_height(gz, lz[indices])
                
                map_x.extend(world_x.tolist())
                map_y.extend(world_y.tolist())
                map_height.extend(actual_height.tolist())
            
            np.savez_compressed(
                filename,
                timestamps=timestamps,
                gantry_x=gantry_x,
                gantry_y=gantry_y,
                gantry_z=gantry_z,
                laser_x=np.array(laser_x_list, dtype=object),
                laser_z=np.array(laser_z_list, dtype=object),
                map_x=np.array(map_x),
                map_y=np.array(map_y),
                map_height=np.array(map_height),
                total_scans=self.total_scans,
                total_points=len(map_x),
                duration=time.time() - self.start_time,
            )
            print(f"💾 Saved {len(self.raw_data)} scans to {filename}")


# ========================
# LITE LIVE PLOT
# ========================
class LitePlot:
    """Minimal live plot: laser line + center point height"""
    
    def __init__(self, recorder):
        self.recorder = recorder
        
        # Create figure
        self.fig, (self.ax_laser, self.ax_height) = plt.subplots(
            1, 2, figsize=(12, 5), facecolor='white'
        )
        
        # Left: Laser profile showing object height (X vs height)
        self.ax_laser.set_xlabel('Laser X (mm)')
        self.ax_laser.set_ylabel('Object Height (mm)')
        self.ax_laser.set_title('Live Object Height Profile')
        self.ax_laser.grid(True, alpha=0.3)
        self.ax_laser.set_xlim(-60, 60)
        self.ax_laser.set_ylim(-50, 200)
        self.line_laser, = self.ax_laser.plot([], [], 'g-', lw=1.5)
        self.center_marker, = self.ax_laser.plot([], [], 'ro', markersize=10, label='Center')
        self.ax_laser.legend(loc='upper right')
        
        # Right: Height history (rolling buffer of center point heights)
        self.ax_height.set_xlabel('Time (s)')
        self.ax_height.set_ylabel('Center Height (mm)')
        self.ax_height.set_title('Center Point Height History')
        self.ax_height.grid(True, alpha=0.3)
        self.line_height, = self.ax_height.plot([], [], 'b-', lw=1)
        
        # Rolling buffer for height history
        self.height_buffer_size = 500
        self.height_times = []
        self.height_values = []
        self.start_time = time.time()
        
        # Current data (updated by external thread)
        self.current_laser_x = np.array([])
        self.current_laser_z = np.array([])
        self.current_actual_h = np.array([])
        self.current_gantry_pos = (0, 0, 0)
        self.laser_lock = threading.Lock()
        
        # Info text
        self.info_text = self.ax_laser.text(
            0.02, 0.98, '', transform=self.ax_laser.transAxes,
            verticalalignment='top', fontsize=9,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )
        
        plt.tight_layout()
        
    def update_laser(self, laser_x, laser_z, actual_h, gantry_pos):
        with self.laser_lock:
            self.current_laser_x = laser_x.copy()
            self.current_laser_z = laser_z.copy()
            self.current_actual_h = actual_h.copy()
            self.current_gantry_pos = gantry_pos
            
    def init_anim(self):
        return [self.line_laser, self.center_marker, self.line_height, self.info_text]
        
    def update_anim(self, frame):
        with self.laser_lock:
            lx = self.current_laser_x
            lz = self.current_laser_z
            actual_h = self.current_actual_h
            gpos = self.current_gantry_pos
        
        gx, gy, gz = gpos
        
        # Update laser line with actual height
        valid = np.isfinite(actual_h) if len(actual_h) > 0 else np.isfinite(lz)
        center_height = None
        center_lz = None
        
        if np.any(valid) and len(actual_h) > 0:
            self.line_laser.set_data(lx[valid], actual_h[valid])
            
            # Auto-scale Y
            h_valid = actual_h[valid]
            h_min, h_max = np.nanmin(h_valid), np.nanmax(h_valid)
            margin = max(10, (h_max - h_min) * 0.1)
            self.ax_laser.set_ylim(h_min - margin, h_max + margin)
            
            # Center point
            center_idx = len(actual_h) // 2
            if np.isfinite(actual_h[center_idx]):
                self.center_marker.set_data([lx[center_idx]], [actual_h[center_idx]])
                center_height = actual_h[center_idx]
                center_lz = lz[center_idx]
                
                # Add to history
                t_now = time.time() - self.start_time
                self.height_times.append(t_now)
                self.height_values.append(center_height)
                
                # Keep buffer size limited
                if len(self.height_times) > self.height_buffer_size:
                    self.height_times = self.height_times[-self.height_buffer_size:]
                    self.height_values = self.height_values[-self.height_buffer_size:]
        
        # Update height history plot
        if len(self.height_times) > 1:
            self.line_height.set_data(self.height_times, self.height_values)
            self.ax_height.set_xlim(max(0, self.height_times[-1] - 30), self.height_times[-1] + 1)
            
            h_arr = np.array(self.height_values)
            h_min, h_max = np.nanmin(h_arr), np.nanmax(h_arr)
            margin = max(0.01, (h_max - h_min) * 0.1)
            self.ax_height.set_ylim(h_min - margin, h_max + margin)
        
        # Update info text
        info = f"Gantry: X={gx:.3f} Y={gy:.3f} Z={gz:.3f}\n"
        info += f"Scans: {self.recorder.total_scans}\n"
        if center_height is not None:
            info += f"Center Height: {center_height:.1f} mm\n"
            info += f"Laser Dist: {center_lz:.1f} mm"
        self.info_text.set_text(info)
        
        return [self.line_laser, self.center_marker, self.line_height, self.info_text]


# ========================
# MAIN FUNCTION
# ========================
def run_lite():
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
    
    # Data recorder
    recorder = DataRecorder()
    
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
                
                # Record data
                recorder.add_scan(laser_x, laser_z, gx, gy, gz)
                
                # Calculate actual height for live display
                actual_h = calculate_actual_height(gz, laser_z)
                
                # Update plotter
                plotter.update_laser(laser_x, laser_z, actual_h, (gx, gy, gz))
            
            # Auto-save
            if (time.time() - last_save) > AUTO_SAVE_INTERVAL_S:
                recorder.save()
                last_save = time.time()
            
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    
    def controller_loop():
        clock = pygame.time.Clock()
        
        while not stop_flag["stop"]:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    stop_flag["stop"] = True
            
            raw_axis_0 = joystick.get_axis(0)
            raw_axis_1 = joystick.get_axis(1)
            raw_axis_3 = joystick.get_axis(3)
            
            axis_x = apply_deadzone(raw_axis_1)
            axis_y = apply_deadzone(-raw_axis_0)
            axis_z = apply_deadzone(raw_axis_3)
            
            speeds["A1"] = axis_y * MAX_SPEED
            speeds["A2"] = axis_x * MAX_SPEED
            speeds["A3"] = axis_z * MAX_SPEED
            
            if joystick.get_button(1):  # B - emergency stop
                speeds["A1"] = speeds["A2"] = speeds["A3"] = 0.0
                print("\n⚠️ Emergency stop!")
            
            if joystick.get_button(7):  # Start - exit
                stop_flag["stop"] = True
            
            if joystick.get_button(2):  # X - save
                recorder.save()
            
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
        plotter = LitePlot(recorder)
        
        print("\n" + "="*60)
        print("GANTRY LASER MAPPER (LITE)")
        print("="*60)
        print("Left Stick:      Move X/Y")
        print("Right Stick Y:   Move Z (laser height)")
        print("B Button:        Emergency stop")
        print("X Button:        Save data")
        print("Start Button:    Exit")
        print("="*60)
        print("Data is being recorded. View later with view_laser_map.py")
        print("="*60 + "\n")
        
        # Start threads
        robot_thread = threading.Thread(target=robot_jog_loop, daemon=True)
        robot_thread.start()
        
        laser_thread = threading.Thread(target=laser_acquisition_loop, args=(grabber, plotter), daemon=True)
        laser_thread.start()
        
        controller_thread = threading.Thread(target=controller_loop, daemon=True)
        controller_thread.start()
        
        # Run animation
        ani = FuncAnimation(
            plotter.fig,
            plotter.update_anim,
            init_func=plotter.init_anim,
            interval=50,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()
        
    finally:
        print("\n=== SHUTTING DOWN ===")
        stop_flag["stop"] = True
        
        # Save data
        if recorder.total_scans > 0:
            recorder.save()
        
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
        print("Exited cleanly.")


if __name__ == "__main__":
    run_lite()
