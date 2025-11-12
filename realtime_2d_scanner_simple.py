"""
Simplified version of real-time 2D scanning visualization.
Uses matplotlib for simpler deployment (no PyQt dependency for visualization).
"""

import sys
import time
import ctypes as ct
import numpy as np
import threading
from collections import deque
from typing import Optional, Tuple

# Add paths for dependencies
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector")
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector\libs\python_bindings")

try:
    import pyllt as llt
except ImportError:
    from pyllt import pyllt as llt

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import Normalize

from libs.cri_lib.cri_controller import CRIController


# ============================================================================
# CONFIGURATION
# ============================================================================
GANTRY_IP = "192.168.3.11"
GANTRY_PORT = 3920

# Gantry offset calibration
GANTRY_X_OFFSET = -0.9
GANTRY_Y_OFFSET = -1.0
GANTRY_Z_OFFSET = 1.3

MAX_SCAN_POINTS = 5000
UPDATE_INTERVAL_MS = 100  # milliseconds


# ============================================================================
# DATA ACQUISITION CLASS
# ============================================================================
class ScanDataAcquisition:
    """Combined data acquisition for laser and gantry."""
    
    def __init__(self):
        # Laser scanner
        self.hLLT = None
        self.scanner_type = None
        self.resolution = 0
        self.laser_connected = False
        
        # Gantry
        self.robot = CRIController()
        self.gantry_connected = False
        
        # Threads
        self.laser_thread = None
        self.gantry_thread = None
        self.running = False
        
        # Scan data
        self.lock = threading.Lock()
        self.scan_x = deque(maxlen=MAX_SCAN_POINTS)
        self.scan_y = deque(maxlen=MAX_SCAN_POINTS)
        self.scan_z = deque(maxlen=MAX_SCAN_POINTS)
        
        # Latest positions
        self.gantry_x = 0.0
        self.gantry_y = 0.0
        self.gantry_z = 0.0
        
        # Buffers for laser
        self.profile_buffer = None
        self.lost_profiles = ct.c_int(0)
        self.x_buf = None
        self.z_buf = None
        self.i_buf = None
        self.snull_us = ct.POINTER(ct.c_ushort)()
        self.snull_ui = ct.POINTER(ct.c_uint)()
    
    def connect_laser(self):
        """Connect to laser scanner."""
        print("Connecting to laser scanner...")
        
        self.hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
        if not self.hLLT:
            raise RuntimeError("Failed to create LLT device")
        
        interfaces = (ct.c_uint * 6)()
        res = llt.get_device_interfaces_fast(self.hLLT, interfaces, len(interfaces))
        if res < 1:
            raise RuntimeError("No laser scanner found")
        
        detected = interfaces[0]
        res = llt.set_device_interface(self.hLLT, detected, 0)
        if res < 1:
            raise RuntimeError("set_device_interface failed")
        
        res = llt.connect(self.hLLT)
        if res < 1:
            raise RuntimeError(f"Connection failed: {res}")
        
        scanner_type = ct.c_int(0)
        llt.get_llt_type(self.hLLT, ct.byref(scanner_type))
        self.scanner_type = scanner_type.value
        
        available_resolutions = (ct.c_uint * 4)()
        llt.get_resolutions(self.hLLT, available_resolutions, len(available_resolutions))
        self.resolution = int(available_resolutions[0])
        llt.set_resolution(self.hLLT, self.resolution)
        llt.set_profile_config(self.hLLT, llt.TProfileConfig.PROFILE)
        
        self.profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        self.x_buf = (ct.c_double * self.resolution)()
        self.z_buf = (ct.c_double * self.resolution)()
        self.i_buf = (ct.c_ushort * self.resolution)()
        
        llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        
        self.laser_connected = True
        print(f"✅ Laser connected (resolution: {self.resolution})")
    
    def connect_gantry(self):
        """Connect to gantry."""
        print("Connecting to gantry...")
        
        if not self.robot.connect(GANTRY_IP, GANTRY_PORT):
            raise RuntimeError("Failed to connect to gantry")
        
        try:
            self.robot.wait_for_status_update(timeout=2.0)
        except Exception:
            pass
        
        self.gantry_connected = True
        print("✅ Gantry connected")
    
    def laser_loop(self):
        """Laser acquisition loop."""
        while self.running:
            try:
                # Get profile
                r = llt.get_actual_profile(
                    self.hLLT,
                    self.profile_buffer,
                    len(self.profile_buffer),
                    llt.TProfileConfig.PROFILE,
                    ct.byref(self.lost_profiles),
                )
                
                if r < 1:
                    time.sleep(0.001)
                    continue
                
                r2 = llt.convert_profile_2_values(
                    self.hLLT, self.profile_buffer, self.resolution,
                    llt.TProfileConfig.PROFILE, self.scanner_type,
                    0, 1, self.snull_us, self.i_buf, self.snull_us,
                    self.x_buf, self.z_buf, self.snull_ui, self.snull_ui,
                )
                
                if r2 < 1:
                    continue
                
                # Convert to numpy
                x_np = np.frombuffer(self.x_buf, dtype=np.float64, count=self.resolution)
                z_np = np.frombuffer(self.z_buf, dtype=np.float64, count=self.resolution)
                i_np = np.frombuffer(self.i_buf, dtype=np.uint16, count=self.resolution)
                
                # Filter valid points
                valid = (i_np > 50) & (z_np > 0) & np.isfinite(z_np)
                
                if valid.any():
                    # Compensate Z with gantry position
                    compensated_z = z_np[valid] - self.gantry_z
                    
                    # Add to scan data
                    with self.lock:
                        for x, z in zip(x_np[valid], compensated_z):
                            self.scan_x.append(x)
                            self.scan_y.append(self.gantry_y)
                            self.scan_z.append(z)
                
            except Exception as e:
                print(f"Laser error: {e}")
            
            time.sleep(0.001)
    
    def gantry_loop(self):
        """Gantry monitoring loop."""
        while self.running:
            try:
                pos = self.robot.robot_state.position_robot
                self.gantry_x = pos.X + GANTRY_X_OFFSET
                self.gantry_y = pos.Y + GANTRY_Y_OFFSET
                self.gantry_z = pos.Z + GANTRY_Z_OFFSET
            except Exception as e:
                print(f"Gantry error: {e}")
            
            time.sleep(0.01)
    
    def start(self):
        """Start acquisition."""
        self.running = True
        
        self.laser_thread = threading.Thread(target=self.laser_loop, daemon=True)
        self.gantry_thread = threading.Thread(target=self.gantry_loop, daemon=True)
        
        self.laser_thread.start()
        self.gantry_thread.start()
        
        print("✅ Acquisition started")
    
    def stop(self):
        """Stop acquisition."""
        self.running = False
        
        if self.laser_thread:
            self.laser_thread.join(timeout=2.0)
        if self.gantry_thread:
            self.gantry_thread.join(timeout=2.0)
        
        if self.laser_connected:
            try:
                llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
                llt.disconnect(self.hLLT)
            except Exception:
                pass
        
        if self.gantry_connected:
            try:
                self.robot.close()
            except Exception:
                pass
        
        print("✅ Acquisition stopped")
    
    def get_scan_data(self):
        """Get current scan data."""
        with self.lock:
            if len(self.scan_x) == 0:
                return None, None, None
            return (
                np.array(self.scan_x),
                np.array(self.scan_y),
                np.array(self.scan_z)
            )
    
    def clear_data(self):
        """Clear scan data."""
        with self.lock:
            self.scan_x.clear()
            self.scan_y.clear()
            self.scan_z.clear()


# ============================================================================
# VISUALIZATION
# ============================================================================
class ScanVisualizer:
    """Matplotlib-based visualization."""
    
    def __init__(self, acquisition: ScanDataAcquisition):
        self.acquisition = acquisition
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.suptitle('Real-time 2D Laser Scan with Height Compensation', fontsize=14)
        
        self.ax.set_xlabel('X Position (mm)', fontsize=12)
        self.ax.set_ylabel('Y Position (mm)', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        
        # Scatter plot
        self.scatter = self.ax.scatter([], [], c=[], cmap='viridis', s=10, vmin=0, vmax=100)
        self.colorbar = self.fig.colorbar(self.scatter, ax=self.ax, label='Height (mm)')
        
        # Info text
        self.info_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )
        
        # Animation
        self.anim = FuncAnimation(
            self.fig, self.update_plot,
            interval=UPDATE_INTERVAL_MS, blit=False
        )
        
        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0
    
    def update_plot(self, frame):
        """Update plot with new data."""
        x, y, z = self.acquisition.get_scan_data()
        
        if x is None or len(x) == 0:
            self.info_text.set_text('Waiting for data...')
            return self.scatter, self.info_text
        
        # Update scatter plot
        self.scatter.set_offsets(np.c_[x, y])
        self.scatter.set_array(z)
        
        # Auto-adjust color limits
        if len(z) > 10:
            z_min, z_max = np.percentile(z, [5, 95])
            z_range = z_max - z_min
            if z_range > 0:
                self.scatter.set_clim(z_min - z_range * 0.1, z_max + z_range * 0.1)
        
        # Update axes limits with some margin
        if len(x) > 10:
            x_margin = (x.max() - x.min()) * 0.1
            y_margin = (y.max() - y.min()) * 0.1
            self.ax.set_xlim(x.min() - x_margin, x.max() + x_margin)
            self.ax.set_ylim(y.min() - y_margin, y.max() + y_margin)
        
        # Calculate FPS
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.last_time)
            self.frame_count = 0
            self.last_time = current_time
        
        # Update info
        gx, gy, gz = self.acquisition.gantry_x, self.acquisition.gantry_y, self.acquisition.gantry_z
        info = (f'Gantry: X={gx:.1f}, Y={gy:.1f}, Z={gz:.1f} mm\n'
                f'Points: {len(x)} | FPS: {self.fps:.1f}')
        self.info_text.set_text(info)
        
        return self.scatter, self.info_text
    
    def show(self):
        """Show the plot."""
        plt.tight_layout()
        plt.show()


# ============================================================================
# MAIN FUNCTION
# ============================================================================
def main():
    """Main entry point."""
    print("=" * 60)
    print("Real-time 2D Laser Scanner (Simplified Version)")
    print("=" * 60)
    
    acquisition = None
    
    try:
        # Initialize acquisition
        acquisition = ScanDataAcquisition()
        acquisition.connect_laser()
        acquisition.connect_gantry()
        acquisition.start()
        
        # Wait for initial data
        print("\nWaiting for initial data...")
        time.sleep(1.0)
        
        # Start visualization
        print("\n✅ Starting visualization")
        print("Close the plot window to exit.\n")
        
        visualizer = ScanVisualizer(acquisition)
        visualizer.show()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if acquisition:
            acquisition.stop()
        print("\n✅ Cleanup complete")


if __name__ == "__main__":
    main()
