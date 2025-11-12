"""
Real-time 2D scanning visualization combining laser scanner and gantry position.
- X-axis: Laser scanner x-position
- Y-axis: Gantry Y position (A2 axis)
- Color: Height (Z from laser - gantry Z position for compensation)
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

try:
    from PyQt5 import QtWidgets, QtCore
except Exception:
    from PySide6 import QtWidgets, QtCore

import pyqtgraph as pg
from libs.cri_lib.cri_controller import CRIController
from libs.cri_lib.robot_state import KinematicsState


# ============================================================================
# CONFIGURATION
# ============================================================================
GANTRY_IP = "192.168.3.11"
GANTRY_PORT = 3920

# Gantry offset calibration (adjust these to match your setup)
GANTRY_X_OFFSET = -0.9  # offset to convert robot X to world X
GANTRY_Y_OFFSET = -1.0  # offset to convert robot Y to world Y
GANTRY_Z_OFFSET = 1.3   # offset to convert robot Z to world Z

# Scanning parameters
MAX_SCAN_POINTS = 10000  # Maximum points to store in memory
UPDATE_RATE_MS = 50      # GUI update rate in milliseconds


# ============================================================================
# LASER SCANNER THREAD
# ============================================================================
class LaserScannerThread(threading.Thread):
    """Thread that continuously acquires laser scanner profiles."""
    
    def __init__(self):
        super().__init__(daemon=True)
        self.hLLT = None
        self.scanner_type = None
        self.resolution = 0
        self.running = False
        self.connected = False
        
        # Profile buffers
        self.profile_buffer = None
        self.lost_profiles = ct.c_int(0)
        self.x = None
        self.z = None
        self.i = None
        self.snull_us = ct.POINTER(ct.c_ushort)()
        self.snull_ui = ct.POINTER(ct.c_uint)()
        
        # Latest valid profile (protected by lock)
        self.lock = threading.Lock()
        self.latest_x = None
        self.latest_z = None
        self.latest_intensity = None
        
    def connect(self):
        """Connect to the laser scanner."""
        print("=== Connecting to Laser Scanner ===")
        
        # Create device handle
        self.hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
        if not self.hLLT:
            raise RuntimeError("Failed to create LLT device handle")
        
        # Search for scanner
        interfaces = (ct.c_uint * 6)()
        res = llt.get_device_interfaces_fast(self.hLLT, interfaces, len(interfaces))
        if res < 1:
            raise RuntimeError("No laser scanner found")
        
        detected = interfaces[0]
        if detected == 0:
            raise RuntimeError("Scanner interface is zero")
        
        print(f"Scanner found: {self._ip_int_to_str(detected)}")
        
        # Set interface
        res = llt.set_device_interface(self.hLLT, detected, 0)
        if res < 1:
            raise RuntimeError(f"set_device_interface failed: {res}")
        
        # Connect
        res = llt.connect(self.hLLT)
        if res < 1:
            raise RuntimeError(f"Connection failed: {res}")
        
        # Get scanner type
        scanner_type = ct.c_int(0)
        res = llt.get_llt_type(self.hLLT, ct.byref(scanner_type))
        if res < 1:
            raise RuntimeError("get_llt_type failed")
        self.scanner_type = scanner_type.value
        
        # Set resolution
        available_resolutions = (ct.c_uint * 4)()
        res = llt.get_resolutions(self.hLLT, available_resolutions, len(available_resolutions))
        if res < 1:
            raise RuntimeError("get_resolutions failed")
        
        self.resolution = int(available_resolutions[0])
        res = llt.set_resolution(self.hLLT, self.resolution)
        if res < 1:
            raise RuntimeError("set_resolution failed")
        
        # Set profile config
        res = llt.set_profile_config(self.hLLT, llt.TProfileConfig.PROFILE)
        if res < 1:
            raise RuntimeError("set_profile_config failed")
        
        # Initialize buffers
        self.profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        self.x = (ct.c_double * self.resolution)()
        self.z = (ct.c_double * self.resolution)()
        self.i = (ct.c_ushort * self.resolution)()
        
        print(f"✅ Laser scanner connected (resolution: {self.resolution})")
        self.connected = True
        
    def start_acquisition(self):
        """Start profile transfer."""
        if not self.connected:
            raise RuntimeError("Scanner not connected")
        
        res = llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        if res < 1:
            raise RuntimeError(f"transfer_profiles start failed: {res}")
        
    def stop_acquisition(self):
        """Stop profile transfer."""
        if self.connected:
            try:
                llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
            except Exception:
                pass
    
    def disconnect(self):
        """Disconnect from scanner."""
        self.stop_acquisition()
        if self.hLLT:
            try:
                llt.disconnect(self.hLLT)
            except Exception:
                pass
            self.connected = False
    
    def grab_profile(self) -> bool:
        """Grab one profile and update latest data."""
        r = llt.get_actual_profile(
            self.hLLT,
            self.profile_buffer,
            len(self.profile_buffer),
            llt.TProfileConfig.PROFILE,
            ct.byref(self.lost_profiles),
        )
        if r < 1:
            return False
        
        r2 = llt.convert_profile_2_values(
            self.hLLT,
            self.profile_buffer,
            self.resolution,
            llt.TProfileConfig.PROFILE,
            self.scanner_type,
            0,  # null points set to 0
            1,  # convert/scale flag
            self.snull_us,
            self.i,
            self.snull_us,
            self.x,
            self.z,
            self.snull_ui,
            self.snull_ui,
        )
        if r2 < 1:
            return False
        
        # Convert to numpy arrays
        x_np = np.frombuffer(self.x, dtype=np.float64, count=self.resolution).copy()
        z_np = np.frombuffer(self.z, dtype=np.float64, count=self.resolution).copy()
        i_np = np.frombuffer(self.i, dtype=np.uint16, count=self.resolution).copy()
        
        # Update latest data (thread-safe)
        with self.lock:
            self.latest_x = x_np
            self.latest_z = z_np
            self.latest_intensity = i_np
        
        return True
    
    def get_latest_profile(self) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """Get the latest profile data (thread-safe)."""
        with self.lock:
            if self.latest_x is None:
                return None
            return self.latest_x.copy(), self.latest_z.copy(), self.latest_intensity.copy()
    
    def run(self):
        """Main acquisition loop."""
        self.running = True
        try:
            self.start_acquisition()
            
            # Wait for first profile
            timeout = time.time() + 5.0
            while self.running and time.time() < timeout:
                if self.grab_profile():
                    print("✅ Laser scanner acquisition started")
                    break
                time.sleep(0.01)
            
            # Main loop
            while self.running:
                self.grab_profile()
                time.sleep(0.001)  # 1ms sleep between grabs
                
        finally:
            self.stop_acquisition()
    
    def stop(self):
        """Stop the acquisition thread."""
        self.running = False
    
    @staticmethod
    def _ip_int_to_str(ip_int: int) -> str:
        return f"{ip_int & 0xFF}.{(ip_int >> 8) & 0xFF}.{(ip_int >> 16) & 0xFF}.{(ip_int >> 24) & 0xFF}"


# ============================================================================
# GANTRY CONTROLLER THREAD
# ============================================================================
class GantryMonitorThread(threading.Thread):
    """Thread that monitors gantry position."""
    
    def __init__(self):
        super().__init__(daemon=True)
        self.robot = CRIController()
        self.running = False
        self.connected = False
        
        # Latest position (protected by lock)
        self.lock = threading.Lock()
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_z = 0.0
    
    def connect(self):
        """Connect to the gantry."""
        print("=== Connecting to Gantry ===")
        
        if not self.robot.connect(GANTRY_IP, GANTRY_PORT):
            raise RuntimeError("Failed to connect to gantry")
        
        # Wait for status
        try:
            self.robot.wait_for_status_update(timeout=2.0)
        except Exception:
            pass
        
        print("✅ Gantry connected")
        self.connected = True
    
    def disconnect(self):
        """Disconnect from gantry."""
        if self.connected:
            try:
                self.robot.close()
            except Exception:
                pass
            self.connected = False
    
    def get_position(self) -> Tuple[float, float, float]:
        """Get current gantry position (world coordinates with compensation)."""
        with self.lock:
            return self.latest_x, self.latest_y, self.latest_z
    
    def run(self):
        """Main monitoring loop."""
        self.running = True
        
        while self.running:
            try:
                # Get robot position
                pos = self.robot.robot_state.position_robot
                
                # Convert to world coordinates with offsets
                x = pos.X + GANTRY_X_OFFSET
                y = pos.Y + GANTRY_Y_OFFSET
                z = pos.Z + GANTRY_Z_OFFSET
                
                # Update latest position (thread-safe)
                with self.lock:
                    self.latest_x = x
                    self.latest_y = y
                    self.latest_z = z
                
            except Exception as e:
                print(f"Gantry position read error: {e}")
            
            time.sleep(0.01)  # 10ms update rate
    
    def stop(self):
        """Stop the monitoring thread."""
        self.running = False


# ============================================================================
# MAIN APPLICATION WINDOW
# ============================================================================
class ScanVisualizerWindow(QtWidgets.QMainWindow):
    """Main application window for 2D scan visualization."""
    
    def __init__(self, laser_thread: LaserScannerThread, gantry_thread: GantryMonitorThread):
        super().__init__()
        self.laser_thread = laser_thread
        self.gantry_thread = gantry_thread
        
        # Data storage
        self.scan_data = {
            'x': deque(maxlen=MAX_SCAN_POINTS),
            'y': deque(maxlen=MAX_SCAN_POINTS),
            'z': deque(maxlen=MAX_SCAN_POINTS),
        }
        
        self.setup_ui()
        
        # Start update timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_visualization)
        self.timer.start(UPDATE_RATE_MS)
        
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0
    
    def setup_ui(self):
        """Setup the user interface."""
        self.setWindowTitle("Real-time 2D Laser Scanner Visualization")
        self.resize(1200, 800)
        
        # Create central widget with layout
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        
        # Info label
        self.info_label = QtWidgets.QLabel("Initializing...")
        self.info_label.setStyleSheet("font-size: 12px; padding: 5px;")
        layout.addWidget(self.info_label)
        
        # Graphics layout for plots
        self.graphics_view = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics_view)
        
        # Create 2D scatter plot
        self.plot = self.graphics_view.addPlot(title="2D Height Map (Top View)")
        self.plot.setLabel('left', 'Y Position (mm)', units='')
        self.plot.setLabel('bottom', 'X Position (mm)', units='')
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setAspectLocked(False)
        
        # Create scatter plot item with color map
        self.scatter = pg.ScatterPlotItem(
            size=3,
            pen=pg.mkPen(None),
            pxMode=True
        )
        self.plot.addItem(self.scatter)
        
        # Add colorbar
        self.colorbar = pg.ColorBarItem(
            values=(0, 100),
            colorMap=pg.colormap.get('viridis'),
            label='Height (mm)',
            limits=(0, 100)
        )
        self.colorbar.setImageItem(self.scatter, insert_in=self.plot)
        
        # Control buttons
        button_layout = QtWidgets.QHBoxLayout()
        
        self.clear_btn = QtWidgets.QPushButton("Clear Data")
        self.clear_btn.clicked.connect(self.clear_data)
        button_layout.addWidget(self.clear_btn)
        
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        button_layout.addWidget(self.pause_btn)
        
        button_layout.addStretch()
        
        self.point_count_label = QtWidgets.QLabel("Points: 0")
        button_layout.addWidget(self.point_count_label)
        
        layout.addLayout(button_layout)
    
    def clear_data(self):
        """Clear all collected scan data."""
        self.scan_data['x'].clear()
        self.scan_data['y'].clear()
        self.scan_data['z'].clear()
        print("Scan data cleared")
    
    def update_visualization(self):
        """Update the visualization with latest data."""
        if self.pause_btn.isChecked():
            return
        
        # Get latest laser profile
        profile = self.laser_thread.get_latest_profile()
        if profile is None:
            self.info_label.setText("⏳ Waiting for laser data...")
            return
        
        laser_x, laser_z, intensity = profile
        
        # Get gantry position
        gantry_x, gantry_y, gantry_z = self.gantry_thread.get_position()
        
        # Filter valid points (intensity > 0 and z is valid)
        valid_mask = (intensity > 50) & (laser_z > 0) & np.isfinite(laser_z)
        
        if not valid_mask.any():
            return
        
        # Extract valid points
        valid_x = laser_x[valid_mask]
        valid_z = laser_z[valid_mask]
        
        # Compensate height: subtract gantry Z position to get object height
        compensated_z = valid_z - gantry_z
        
        # Add points to scan data
        # X position: laser x coordinate
        # Y position: gantry y position (same for all points in this profile)
        # Z value: compensated height
        for x, z in zip(valid_x, compensated_z):
            self.scan_data['x'].append(x)
            self.scan_data['y'].append(gantry_y)
            self.scan_data['z'].append(z)
        
        # Convert to numpy arrays for plotting
        if len(self.scan_data['x']) == 0:
            return
        
        x_arr = np.array(self.scan_data['x'])
        y_arr = np.array(self.scan_data['y'])
        z_arr = np.array(self.scan_data['z'])
        
        # Update scatter plot
        # Create color map based on height
        colors = z_arr.copy()
        
        # Update colorbar range dynamically
        if len(z_arr) > 0:
            z_min, z_max = np.percentile(z_arr, [5, 95])
            z_range = z_max - z_min
            if z_range > 0:
                z_min -= z_range * 0.1
                z_max += z_range * 0.1
                self.colorbar.setLevels(values=(z_min, z_max))
        
        # Create spots for scatter plot
        spots = [{'pos': (x, y), 'data': 1} for x, y in zip(x_arr, y_arr)]
        
        # Update scatter plot
        self.scatter.setData(
            pos=np.column_stack([x_arr, y_arr]),
            brush=[pg.mkBrush(color) for color in self.get_colors(colors)]
        )
        
        # Update info
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
        
        self.info_label.setText(
            f"📊 Gantry: X={gantry_x:.1f}, Y={gantry_y:.1f}, Z={gantry_z:.1f} mm | "
            f"Points: {len(self.scan_data['x'])} | FPS: {self.fps:.1f}"
        )
        self.point_count_label.setText(f"Points: {len(self.scan_data['x'])}")
    
    def get_colors(self, values):
        """Convert height values to colors using colormap."""
        if len(values) == 0:
            return []
        
        # Normalize values to 0-1 range
        v_min, v_max = self.colorbar.levels()
        normalized = np.clip((values - v_min) / (v_max - v_min + 1e-10), 0, 1)
        
        # Get colormap
        cmap = pg.colormap.get('viridis')
        colors = cmap.map(normalized, mode='qcolor')
        
        return colors
    
    def closeEvent(self, event):
        """Handle window close event."""
        self.timer.stop()
        event.accept()


# ============================================================================
# MAIN FUNCTION
# ============================================================================
def main():
    """Main application entry point."""
    print("=" * 60)
    print("Real-time 2D Laser Scanner with Gantry Compensation")
    print("=" * 60)
    
    app = QtWidgets.QApplication(sys.argv)
    
    laser_thread = None
    gantry_thread = None
    window = None
    
    try:
        # Initialize and connect laser scanner
        laser_thread = LaserScannerThread()
        laser_thread.connect()
        laser_thread.start()
        
        # Initialize and connect gantry
        gantry_thread = GantryMonitorThread()
        gantry_thread.connect()
        gantry_thread.start()
        
        # Wait a bit for threads to initialize
        time.sleep(0.5)
        
        # Create and show main window
        window = ScanVisualizerWindow(laser_thread, gantry_thread)
        window.show()
        
        print("\n✅ Application started successfully!")
        print("Move the gantry to scan objects.")
        print("The visualization shows X (laser) vs Y (gantry) with height as color.")
        print("\nClose the window to exit.")
        
        # Run application
        app.exec_()
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print("\nShutting down...")
        
        if laser_thread:
            laser_thread.stop()
            laser_thread.join(timeout=2.0)
            laser_thread.disconnect()
        
        if gantry_thread:
            gantry_thread.stop()
            gantry_thread.join(timeout=2.0)
            gantry_thread.disconnect()
        
        print("✅ Cleanup complete")


if __name__ == "__main__":
    main()
