# combined_laser_xbox.py
# Combines laser scanning/plotting with Xbox controller robot control.
# Saves Laser Data + Robot Position to CSV.

import sys
import time
import ctypes as ct
import numpy as np
import scipy as sp
import signal
import csv
import json
import threading
import queue
from datetime import datetime
from contextlib import contextmanager

# --- PATH SETUP ---
# Adjust these paths as per the original scripts
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector\libs\python_bindings")
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector")

# --- IMPORTS ---
try:
    import pyllt as llt
except ImportError:
    try:
        from pyllt import pyllt as llt
    except Exception as e:
        print("ERROR: Could not import pyllt.")
        raise

try:
    import pygame
except ImportError:
    print("pygame not installed. Please run: pip install pygame")
    sys.exit(1)

import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
from matplotlib.animation import FuncAnimation

from libs.cri_lib.cri_controller import CRIController
from libs.cri_lib.robot_state import KinematicsState

# --- ROBOT CONSTANTS ---
ROBOT_IP = "192.168.3.11"
ROBOT_PORT = 3920
JOG_UPDATE_HZ = 50
JOG_DT = 1.0 / JOG_UPDATE_HZ
MAX_SPEED = 150.0
DEADZONE = 0.05

# --- SHARED STATE ---
speeds = {"A1": 0.0, "A2": 0.0, "A3": 0.0}
stop_flag = {"stop": False}
robot_ready = {"ready": False}

# --- HELPERS ---
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

def apply_deadzone(value, deadzone=DEADZONE):
    if abs(value) < deadzone:
        return 0.0
    sign = 1 if value > 0 else -1
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return sign * scaled

def robot_jog_loop(robot):
    """Send jog commands to robot continuously in a separate thread"""
    while not stop_flag["stop"]:
        if robot_ready["ready"]:
            robot.set_jog_values(
                A1=speeds["A1"],
                A2=speeds["A2"],
                A3=speeds["A3"],
                A4=0.0, A5=0.0, A6=0.0,
                E1=0.0, E2=0.0, E3=0.0,
            )
        time.sleep(JOG_DT)

# --- LASER SETUP ---
def connect_scanner() -> tuple:
    print("=== CREATE HANDLE ===")
    hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
    if not hLLT:
        raise RuntimeError("Failed to create LLT device handle")

    print("=== SEARCH INTERFACES ===")
    interfaces = (ct.c_uint * 6)()
    res = llt.get_device_interfaces_fast(hLLT, interfaces, len(interfaces))
    if not check_llt_success(res, "get_device_interfaces_fast"):
        raise RuntimeError("No interfaces found")

    detected = interfaces[0]
    if detected == 0:
        raise RuntimeError("Interface slot is zero (no scanner).")

    print(f"Scanner detected at: {ip_int_to_str(detected)} (0x{detected:08X})")
    print("=== CONFIGURE INTERFACE ===")
    res = llt.set_device_interface(hLLT, detected, 0)
    if not check_llt_success(res, "set_device_interface"):
        raise RuntimeError(f"set_device_interface failed: {res}")

    print("=== CONNECT ===")
    res = llt.connect(hLLT)
    if not check_llt_success(res, "connect"):
        if res == -301:
            raise RuntimeError("Scanner in use by another software (-301). Close config tools.")
        elif res == -303:
            raise RuntimeError("Handle already in use (-303). Restart Python.")
        else:
            raise RuntimeError(f"Connection failed: {res}")

    print("🎉 Connected.")
    return hLLT, detected

def setup_scanner(hLLT) -> tuple:
    print("=== SCANNER SETUP ===")
    scanner_type = ct.c_int(0)
    res = llt.get_llt_type(hLLT, ct.byref(scanner_type))
    if not check_llt_success(res, "get_llt_type"):
        raise RuntimeError("get_llt_type failed")
    print(f"Scanner type: {scanner_type.value}")

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

    # Set Idle Time to 2000us (500Hz)
    try:
        # Check if FEATURE_FUNCTION_IDLETIME exists
        idle_time_id = getattr(llt, 'FEATURE_FUNCTION_IDLETIME', None)
        if idle_time_id is None:
             idle_time_id = getattr(llt, 'FEATURE_FUNCTION_IDLE_TIME', None)
             
        if idle_time_id is not None:
            # 2000 microseconds = 2ms = 500Hz
            res = llt.set_feature(hLLT, idle_time_id, 2000)
            check_llt_success(res, "set_feature(IDLETIME, 2000)")
        else:
            print("WARNING: FEATURE_FUNCTION_IDLETIME not found in pyllt.")
    except Exception as e:
        print(f"WARNING: Could not set idle time: {e}")

    return scanner_type, resolution

    return scanner_type, resolution

# --- ACQUISITION THREAD ---
class AcquisitionThread(threading.Thread):
    def __init__(self, hLLT, scanner_type, resolution, robot):
        super().__init__()
        self.hLLT = hLLT
        self.scanner_type = scanner_type
        self.resolution = int(resolution)
        self.robot = robot
        self.running = True
        self.data_queue = queue.Queue(maxsize=1)

        # Buffers
        self.profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        self.lost_profiles = ct.c_int(0)
        self.x = (ct.c_double * self.resolution)()
        self.z = (ct.c_double * self.resolution)()
        self.i = (ct.c_ushort * self.resolution)()
        self.snull_us = ct.POINTER(ct.c_ushort)()
        self.snull_ui = ct.POINTER(ct.c_uint)()
        
        self.rotation_angle = -0.15
        self.frames_seen = 0

        # CSV Setup
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"scan_data_combined_{timestamp}.csv"
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["frame_num", "min_z", "max_z", "robot_x", "robot_y", "robot_z", "x_values", "z_values"])
        print(f"Saving data to {self.csv_filename}")

    def rotate_points(self, x, z, angle_deg):
        if x.size == 0 or z.size == 0: return x, z
        x_center = np.mean(x)
        z_center = np.mean(z)
        angle_rad = np.radians(angle_deg)
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)
        x_trans = x - x_center
        z_trans = z - z_center
        x_rot = x_trans * cos_a - z_trans * sin_a
        z_rot = x_trans * sin_a + z_trans * cos_a
        return x_rot + x_center, z_rot + z_center

    def run(self):
        while self.running:
            # 1. Get Laser Data
            r = llt.get_actual_profile(
                self.hLLT, self.profile_buffer, len(self.profile_buffer),
                llt.TProfileConfig.PROFILE, ct.byref(self.lost_profiles),
            )
            if r < 1: continue

            r2 = llt.convert_profile_2_values(
                self.hLLT, self.profile_buffer, self.resolution, llt.TProfileConfig.PROFILE,
                self.scanner_type, 0, 1, self.snull_us, self.i, self.snull_us, self.x, self.z, self.snull_ui, self.snull_ui,
            )
            if r2 < 1: continue

            x_np = np.frombuffer(self.x, dtype=np.float64, count=self.resolution).copy()
            z_np = np.frombuffer(self.z, dtype=np.float64, count=self.resolution).copy()
            i_np = np.frombuffer(self.i, dtype=np.uint16, count=self.resolution).copy()

            invalid = (i_np == 0) | (~np.isfinite(z_np)) | (z_np <= 0)
            if invalid.any(): z_np[invalid] = np.nan
            
            self.frames_seen += 1

            # 2. Get Robot Position (Fast as possible)
            rx, ry, rz = 0.0, 0.0, 0.0
            try:
                # Accessing robot_state should be thread-safe enough for reading
                pos = self.robot.robot_state.position_robot
                rx = pos.X - 0.9
                ry = pos.Y - 1.0
                rz = pos.Z + 1.3
            except:
                pass

            # 3. Process & Save
            valid = np.isfinite(z_np)
            xv = x_np[valid]
            zv = z_np[valid]

            if self.rotation_angle != 0.0:
                xv, zv = self.rotate_points(xv, zv, self.rotation_angle)

            if xv.size > 0:
                current_min_z = float(np.min(zv))
                current_max_z = float(np.max(zv))
                self.csv_writer.writerow([
                    self.frames_seen,
                    current_min_z,
                    current_max_z,
                    rx, ry, rz,
                    json.dumps(xv.tolist()),
                    json.dumps(zv.tolist())
                ])

            # 4. Send to Plotter
            if self.data_queue.full():
                try: self.data_queue.get_nowait()
                except queue.Empty: pass
            self.data_queue.put((xv, zv, self.lost_profiles.value))

    def stop(self):
        self.running = False
        self.join()
        self.csv_file.close()
        print(f"Closed CSV file: {self.csv_filename}")

# --- LIVE PLOT CLASS ---
class LivePlot:
    def __init__(self, acquisition_thread, robot, joystick):
        self.thread = acquisition_thread
        self.robot = robot
        self.joystick = joystick

        # Plot setup
        plt.rcParams.update({
            "font.size": 18, "axes.labelsize": 20, "axes.titlesize": 22,
            "xtick.labelsize": 18, "ytick.labelsize": 18, "legend.fontsize": 18, "lines.linewidth": 2,
        })
        self.fig, self.ax_single = plt.subplots(figsize=(10, 6), facecolor="white")
        self.ax_single.grid(True)
        self.ax_single.set_xlabel("x (mm)", fontsize=20)
        self.ax_single.set_ylabel("z (mm)", fontsize=20)
        (self.line_single,) = self.ax_single.plot([], [], "g-", lw=2)
        (self.line_max,) = self.ax_single.plot([], [], "r--", lw=2, label="max")
        (self.line_min,) = self.ax_single.plot([], [], "b--", lw=2, label="min")
        self.ax_single.legend(loc="upper right", fontsize=18)
        
        for axis in (self.ax_single.xaxis, self.ax_single.yaxis):
            fmt = ScalarFormatter(useOffset=False)
            fmt.set_scientific(False)
            axis.set_major_formatter(fmt)
        self.ax_single.tick_params(axis="both", which="major", labelsize=18)

        self.global_z_min = np.inf
        self.global_z_max = -np.inf
        self.global_x_min = np.inf
        self.global_x_max = -np.inf

    def init(self):
        self.line_single.set_data([], [])
        self.line_max.set_data([], [])
        self.line_min.set_data([], [])
        return [self.line_single, self.line_max, self.line_min]

    def update(self, _):
        # --- 1. Handle Xbox Controller ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stop_flag["stop"] = True
                # We can't easily close the plot from here without error, but we can signal stop
        
        if self.joystick:
            raw_axis_0 = self.joystick.get_axis(0)
            raw_axis_1 = self.joystick.get_axis(1)
            raw_axis_3 = self.joystick.get_axis(3)
            
            axis_x = apply_deadzone(raw_axis_1)
            axis_y = apply_deadzone(-raw_axis_0)
            axis_z = apply_deadzone(raw_axis_3)
            
            speeds["A1"] = axis_y * MAX_SPEED
            speeds["A2"] = axis_x * MAX_SPEED
            speeds["A3"] = axis_z * MAX_SPEED
            
            if self.joystick.get_button(1): # B button
                speeds["A1"] = speeds["A2"] = speeds["A3"] = 0.0
                print("\nEmergency stop activated!")
            
            if self.joystick.get_button(7): # Start button
                stop_flag["stop"] = True

        # --- 2. Get Laser Data from Thread ---
        try:
            data = self.thread.data_queue.get_nowait()
        except queue.Empty:
            return self.init()

        xv, zv, lost = data

        # --- 3. Plotting ---
        order = np.argsort(xv)
        xv_sorted = xv[order]
        zv_sorted = zv[order]
        try:
            spline = sp.interpolate.make_smoothing_spline(xv_sorted, zv_sorted, lam=20)
            zv_spl_sorted = spline(xv_sorted)
            zv_spl = np.empty_like(zv)
            zv_spl[order] = zv_spl_sorted
        except Exception:
            zv_spl = zv # Fallback if spline fails

        self.line_single.set_data(xv, zv_spl)

        if zv_spl.size > 0:
            z_max = float(np.nanmax(zv_spl))
            z_min = float(np.nanmin(zv_spl))
            x_range = [xv.min(), xv.max()]
            self.line_max.set_data(x_range, [z_max, z_max])
            self.line_min.set_data(x_range, [z_min, z_min])

        if xv.size and zv.size:
            self.global_x_min = min(self.global_x_min, float(np.nanmin(xv)))
            self.global_x_max = max(self.global_x_max, float(np.nanmax(xv)))
            self.global_z_min = min(self.global_z_min, float(np.nanmin(zv)))
            self.global_z_max = max(self.global_z_max, float(np.nanmax(zv)))
            x_span = max(1e-6, self.global_x_max - self.global_x_min)
            z_span = max(1e-6, self.global_z_max - self.global_z_min)
            xm = x_span * 0.05
            zm = z_span * 0.05
            self.ax_single.set_xlim(self.global_x_min - xm, self.global_x_max + xm)
            self.ax_single.set_ylim(self.global_z_min - zm, self.global_z_max + zm)
            
        return [self.line_single, self.line_max, self.line_min]

def run(interval_ms=30, timeout_first_frame=5.0):
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # --- INIT PYGAME ---
    pygame.init()
    pygame.joystick.init()
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Controller connected: {joystick.get_name()}")
    else:
        print("Warning: No Xbox controller detected.")

    # --- INIT ROBOT ---
    robot = CRIController()
    active = enabled = False
    jog_active = False
    
    try:
        print("Connecting to robot...")
        if robot.connect(ROBOT_IP, ROBOT_PORT):
            print("Robot connected.")
            try:
                robot.wait_for_status_update(timeout=2.0)
            except: pass
            
            active = robot.set_active_control(True)
            
            # Wait for ready
            ready_deadline = time.time() + 10.0
            while time.time() < ready_deadline:
                robot.wait_for_status_update(timeout=1.0)
                with robot.robot_state_lock:
                    rs = robot.robot_state
                    kin_ok = (rs.kinematics_state == KinematicsState.NO_ERROR and
                              rs.combined_axes_error == "NoError")
                if kin_ok: break
                robot.reset()
            
            enabled = robot.enable()
            if enabled:
                robot.start_jog()
                jog_active = True
                robot_ready["ready"] = True
                
                # Start jog thread
                t = threading.Thread(target=robot_jog_loop, args=(robot,), daemon=True)
                t.start()
                print("Robot control active.")
            else:
                print("Robot enable failed.")
        else:
            print("Robot connection failed.")

        # --- INIT LASER ---
        hLLT = None
        thread = None
        try:
            hLLT, _ = connect_scanner()
            scanner_type, resolution = setup_scanner(hLLT)

            thread = AcquisitionThread(hLLT, scanner_type, resolution, robot)
            live = LivePlot(thread, robot, joystick)

            with safe_transfer(hLLT):
                thread.start()
                
                t0 = time.time()
                first_ok = False
                while time.time() - t0 < timeout_first_frame:
                    if not thread.data_queue.empty():
                        first_ok = True
                        break
                    time.sleep(0.01)

                if not first_ok:
                    print("ERROR: No profiles received.")
                    return

                ani = FuncAnimation(
                    live.fig,
                    live.update,
                    init_func=live.init,
                    interval=interval_ms,
                    blit=True,
                )
                
                # Keyboard handler for rotation
                def on_key(event):
                    if event.key == 'left':
                        thread.rotation_angle -= 0.01
                        print(f"Rotation angle: {thread.rotation_angle:.2f}°")
                    elif event.key == 'right':
                        thread.rotation_angle += 0.01
                        print(f"Rotation angle: {thread.rotation_angle:.2f}°")
                live.fig.canvas.mpl_connect('key_press_event', on_key)

                print("Starting main loop. Close window or press Start on controller to exit.")
                plt.show()

        finally:
            if thread is not None: thread.stop()
            if hLLT is not None: llt.disconnect(hLLT)

    finally:
        stop_flag["stop"] = True
        print("Shutting down robot...")
        if jog_active: robot.stop_jog()
        if enabled: robot.disable()
        if active: robot.set_active_control(False)
        robot.close()
        pygame.quit()
        print("Done.")

if __name__ == "__main__":
    run(interval_ms=50)
