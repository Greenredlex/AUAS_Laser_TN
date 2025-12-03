"""Combined WASDQE gantry jog + scanCONTROL height mapping.

This script fuses the existing CRI gantry jog controller with the MICRO-EPSILON
scanCONTROL profile acquisition so you can manually move the gantry (WASDQE)
while continuously stitching the laser line profiles into a live 2D height map.

Run this file to:
- Hold W/S for gantry axis A1, A/D for A2, Q/E for A3 (same as the standalone jogger)
- Stream line profiles (~10 cm along X) from the laser
- Convert each line into absolute (X, Y) surface points with a color-coded height
- Watch the map fill in as you sweep the gantry across the target

Dependencies: keyboard, numpy, pyqtgraph, pyllt (LLT.dll bindings), CRI libs.
"""

from __future__ import annotations

import ctypes as ct
import signal
import sys
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Optional, Tuple

import keyboard
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# Project-specific libraries (adjust the paths if your workspace differs)
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector")
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector\libs\python_bindings")

try:
    import pyllt as llt
except ImportError:  # pragma: no cover - fallback for alternate packaging
    from pyllt import pyllt as llt  # type: ignore

from libs.cri_lib.cri_controller import CRIController
from libs.cri_lib.robot_state import KinematicsState

# ---------------------------------------------------------------------------
# Configuration knobs (tweak to match your rig)
# ---------------------------------------------------------------------------
ROBOT_IP = "192.168.3.11"
ROBOT_PORT = 3920

# Jog behaviour
JOG_SPEED_STEP = 50.0       # percent of max per axis when the key is held
JOG_UPDATE_HZ = 50.0        # how often to send jog commands
JOG_DT = 1.0 / JOG_UPDATE_HZ
KEYBOARD_POLL_DT = 0.01

# Convert CRI Cartesian pose (meters) into mm + offsets (empirical from earlier scripts)
POSE_OFFSETS_M = {
    "X": -0.9,   # shift so 0,0,0 roughly matches table origin
    "Y": -1.0,
    "Z": +1.3,
}
POSE_SCALE_TO_MM = 1000.0

# Laser line alignment relative to gantry pose
LASER_LINE_FLIP_X = 1.0          # set to -1.0 if scanCONTROL X runs opposite the gantry +X
LASER_LINE_OFFSET_X_MM = 0.0     # extra fixed offset from gantry X to the laser origin
LASER_LINE_OFFSET_Y_MM = 0.0     # lateral offset if the laser is displaced from gantry Y

# Height fusion: card position axis that represents the physical laser height
LASER_HEIGHT_AXIS = "Y"          # which Pose axis ("X", "Y" or "Z") tracks the laser height
LASER_HEIGHT_SCALE = -1.0        # -1.0 because "cardpos gives Y in inverse"
LASER_HEIGHT_OFFSET_MM = 0.0     # add a fixed offset to convert to absolute table height

# Keep a finite history to avoid unbounded RAM usage while allowing >400k points
MAX_POINTS_IN_MAP = 2_000_000

# Visualisation tuning
DISPLAY_POINT_BUDGET = 800_000    # limit per-frame sample sent to GPU for rendering
PLOT_REFRESH_MS = 40              # GUI timer interval in ms
AXIS_PADDING_MM = 15.0
SCATTER_DIAMETER_PX = 4.0


# ---------------------------------------------------------------------------
# Low-level helpers reused from the standalone scan script
# ---------------------------------------------------------------------------
def check_llt_success(result: int, operation: str) -> bool:
    if result >= 1:
        print(f"✅ {operation}: {result}")
        return True
    print(f"❌ {operation}: {result}")
    return False


def ip_int_to_str(ip_int: int) -> str:
    return f"{ip_int & 0xFF}.{(ip_int >> 8) & 0xFF}.{(ip_int >> 16) & 0xFF}.{(ip_int >> 24) & 0xFF}"


@contextmanager
def transfer_profiles_guard(hLLT):
    ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
    if ret < 1:
        raise RuntimeError(f"transfer_profiles start failed: {ret}")
    try:
        yield
    finally:
        try:
            llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
        except Exception:
            pass


@dataclass
class PoseMM:
    x_mm: float
    y_mm: float
    z_mm: float


class LaserScanner:
    """Thin wrapper around the pyllt helper calls."""

    def __init__(self):
        self.hLLT = None
        self.scanner_type = None
        self.resolution = None
        self.profile_buffer = None
        self.lost_profiles = ct.c_int(0)
        self.x = None
        self.z = None
        self.i = None
        self.snull_us = ct.POINTER(ct.c_ushort)()
        self.snull_ui = ct.POINTER(ct.c_uint)()

    # Connection/setup -----------------------------------------------------
    def connect(self):
        print("=== CONNECT LASER ===")
        hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
        if not hLLT:
            raise RuntimeError("create_llt_device failed")

        interfaces = (ct.c_uint * 6)()
        res = llt.get_device_interfaces_fast(hLLT, interfaces, len(interfaces))
        if not check_llt_success(res, "get_device_interfaces_fast"):
            raise RuntimeError("No interfaces found")

        detected = interfaces[0]
        if detected == 0:
            raise RuntimeError("Interface slot empty (no scanner)")
        print(f"Scanner detected @ {ip_int_to_str(detected)}")

        res = llt.set_device_interface(hLLT, detected, 0)
        if not check_llt_success(res, "set_device_interface"):
            raise RuntimeError("set_device_interface failed")

        res = llt.connect(hLLT)
        if not check_llt_success(res, "connect"):
            raise RuntimeError("Scanner connect failed")

        self.hLLT = hLLT
        return hLLT

    def configure(self):
        if not self.hLLT:
            raise RuntimeError("Scanner not connected")
        scanner_type = ct.c_int(0)
        res = llt.get_llt_type(self.hLLT, ct.byref(scanner_type))
        if not check_llt_success(res, "get_llt_type"):
            raise RuntimeError("get_llt_type failed")

        available_res = (ct.c_uint * 4)()
        res = llt.get_resolutions(self.hLLT, available_res, len(available_res))
        if not check_llt_success(res, "get_resolutions"):
            raise RuntimeError("get_resolutions failed")
        res_list = [int(v) for v in available_res if int(v) > 0]
        resolution = max(res_list) if res_list else int(available_res[0])
        res = llt.set_resolution(self.hLLT, resolution)
        if not check_llt_success(res, "set_resolution"):
            raise RuntimeError("set_resolution failed")

        res = llt.set_profile_config(self.hLLT, llt.TProfileConfig.PROFILE)
        if not check_llt_success(res, "set_profile_config(PROFILE)"):
            raise RuntimeError("set_profile_config failed")

        self.scanner_type = scanner_type
        self.resolution = int(resolution)
        self.profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        self.x = (ct.c_double * self.resolution)()
        self.z = (ct.c_double * self.resolution)()
        self.i = (ct.c_ushort * self.resolution)()
        print(f"Laser ready (type={scanner_type.value}, res={self.resolution})")

    # Profile acquisition --------------------------------------------------
    def grab_profile(self) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, int]]:
        if not self.hLLT:
            return None
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
            0,
            1,
            self.snull_us,
            self.i,
            self.snull_us,
            self.x,
            self.z,
            self.snull_ui,
            self.snull_ui,
        )
        if r2 < 1:
            return None

        x_np = np.frombuffer(self.x, dtype=np.float64, count=self.resolution).copy()
        z_np = np.frombuffer(self.z, dtype=np.float64, count=self.resolution).copy()
        i_np = np.frombuffer(self.i, dtype=np.uint16, count=self.resolution).copy()
        invalid = (~np.isfinite(z_np)) | (z_np <= 0) | (i_np == 0)
        if invalid.any():
            z_np[invalid] = np.nan
        return x_np, z_np, i_np, self.lost_profiles.value

    def disconnect(self):
        if self.hLLT:
            try:
                llt.disconnect(self.hLLT)
            except Exception:
                pass
            self.hLLT = None


class GantryJogController:
    """Replicates the standalone jogging behaviour with pose tracking."""

    def __init__(self):
        self.robot = CRIController()
        self.active = False
        self.enabled = False
        self.jog_active = False
        self.speeds = {"A1": 0.0, "A2": 0.0, "A3": 0.0}
        self.stop_evt = threading.Event()
        self.pose_lock = threading.Lock()
        self.latest_pose: Optional[PoseMM] = None
        self.keyboard_thread: Optional[threading.Thread] = None
        self.jog_thread: Optional[threading.Thread] = None

    # Public API -----------------------------------------------------------
    def start(self):
        if not self._connect_and_enable():
            raise RuntimeError("Failed to prepare robot")
        self.keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.keyboard_thread.start()
        self.jog_thread = threading.Thread(target=self._jog_loop, daemon=True)
        self.jog_thread.start()

    def stop(self):
        self.stop_evt.set()
        if self.keyboard_thread:
            self.keyboard_thread.join(timeout=1.0)
        if self.jog_thread:
            self.jog_thread.join(timeout=1.0)
        self._shutdown_robot()

    def get_pose(self) -> Optional[PoseMM]:
        with self.pose_lock:
            return self.latest_pose

    # Internals ------------------------------------------------------------
    def _connect_and_enable(self) -> bool:
        if not self.robot.connect(ROBOT_IP, ROBOT_PORT):
            print("Connect failed")
            return False
        try:
            self.robot.wait_for_status_update(timeout=2.0)
        except Exception:
            pass

        self.active = self.robot.set_active_control(True)
        ready_deadline = time.time() + 10.0
        while time.time() < ready_deadline:
            self.robot.wait_for_status_update(timeout=1.0)
            with self.robot.robot_state_lock:
                rs = self.robot.robot_state
                kin_ok = (
                    rs.kinematics_state == KinematicsState.NO_ERROR
                    and rs.combined_axes_error == "NoError"
                )
            if kin_ok:
                break
            self.robot.reset()

        self.enabled = self.robot.enable()
        if not self.enabled:
            print("Enable failed")
            return False
        self.robot.start_jog()
        self.jog_active = True
        print("Gantry ready. Hold WASD/QE, SPACE zeroes, ESC exits.")
        return True

    def _keyboard_loop(self):
        while not self.stop_evt.is_set():
            if keyboard.is_pressed("esc"):
                self.stop_evt.set()
                break
            if keyboard.is_pressed("space"):
                self.speeds["A1"] = self.speeds["A2"] = self.speeds["A3"] = 0.0
            self.speeds["A1"] = +JOG_SPEED_STEP if keyboard.is_pressed("w") else (
                -JOG_SPEED_STEP if keyboard.is_pressed("s") else 0.0
            )
            self.speeds["A2"] = +JOG_SPEED_STEP if keyboard.is_pressed("d") else (
                -JOG_SPEED_STEP if keyboard.is_pressed("a") else 0.0
            )
            self.speeds["A3"] = +JOG_SPEED_STEP if keyboard.is_pressed("q") else (
                -JOG_SPEED_STEP if keyboard.is_pressed("e") else 0.0
            )
            time.sleep(KEYBOARD_POLL_DT)

    def _jog_loop(self):
        while not self.stop_evt.is_set():
            self.robot.set_jog_values(
                A1=self.speeds["A1"],
                A2=self.speeds["A2"],
                A3=self.speeds["A3"],
                A4=0.0,
                A5=0.0,
                A6=0.0,
                E1=0.0,
                E2=0.0,
                E3=0.0,
            )
            with self.robot.robot_state_lock:
                pos = self.robot.robot_state.position_robot
                pose = PoseMM(
                    x_mm=(pos.X + POSE_OFFSETS_M["X"]) * POSE_SCALE_TO_MM,
                    y_mm=(pos.Y + POSE_OFFSETS_M["Y"]) * POSE_SCALE_TO_MM,
                    z_mm=(pos.Z + POSE_OFFSETS_M["Z"]) * POSE_SCALE_TO_MM,
                )
            with self.pose_lock:
                self.latest_pose = pose
            time.sleep(JOG_DT)

    def _shutdown_robot(self):
        try:
            if self.jog_active:
                self.robot.stop_jog()
        except Exception:
            pass
        try:
            self.robot.stop_move()
        except Exception:
            pass
        if self.enabled:
            try:
                self.robot.disable()
            except Exception:
                pass
        if self.active:
            try:
                self.robot.set_active_control(False)
            except Exception:
                pass
        try:
            self.robot.close()
        except Exception:
            pass
        print("Gantry disconnected")


class PointRingBuffer:
    """Thread-safe circular buffer that can be sampled without reallocations."""

    def __init__(self, capacity: int = MAX_POINTS_IN_MAP):
        self.capacity = capacity
        self._lock = threading.Lock()
        self._x = np.empty(capacity, dtype=np.float32)
        self._y = np.empty(capacity, dtype=np.float32)
        self._h = np.empty(capacity, dtype=np.float32)
        self._head = 0
        self._count = 0
        self._dirty = False

    def add_points(self, x_vals: np.ndarray, y_vals: np.ndarray, h_vals: np.ndarray) -> int:
        n = int(x_vals.size)
        if n == 0:
            return 0
        if n > self.capacity:
            # Keep only the newest samples if we overflow the buffer in one batch.
            x_vals = x_vals[-self.capacity :]
            y_vals = y_vals[-self.capacity :]
            h_vals = h_vals[-self.capacity :]
            n = self.capacity
        with self._lock:
            first = min(n, self.capacity - self._head)
            second = n - first
            self._x[self._head : self._head + first] = x_vals[:first]
            self._y[self._head : self._head + first] = y_vals[:first]
            self._h[self._head : self._head + first] = h_vals[:first]
            if second:
                self._x[:second] = x_vals[first:]
                self._y[:second] = y_vals[first:]
                self._h[:second] = h_vals[first:]
            self._head = (self._head + n) % self.capacity
            self._count = min(self.capacity, self._count + n)
            self._dirty = True
            return n

    def snapshot(
        self, max_points: Optional[int] = DISPLAY_POINT_BUDGET
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        with self._lock:
            self._dirty = False
            count = self._count
            if count == 0:
                empty = np.empty(0, dtype=np.float32)
                return empty, empty, empty
            start = (self._head - count) % self.capacity
            if start < self._head:
                x = self._x[start : start + count].copy()
                y = self._y[start : start + count].copy()
                h = self._h[start : start + count].copy()
            else:
                x = np.concatenate((self._x[start:], self._x[: self._head])).copy()
                y = np.concatenate((self._y[start:], self._y[: self._head])).copy()
                h = np.concatenate((self._h[start:], self._h[: self._head])).copy()

        if max_points and count > max_points:
            idx = np.linspace(0, count - 1, num=max_points, dtype=np.int32)
            return x[idx], y[idx], h[idx]
        return x, y, h

    def has_fresh_data(self) -> bool:
        with self._lock:
            return self._dirty

    def point_count(self) -> int:
        with self._lock:
            return self._count


class LiveHeightMapPlotter(QtWidgets.QMainWindow):
    """PyQtGraph-based scatter renderer capable of hundreds of thousands of points."""

    def __init__(self, aggregator: PointRingBuffer, stop_evt: threading.Event):
        app = QtWidgets.QApplication.instance()
        if app is None:
            app = QtWidgets.QApplication(sys.argv)
        self.app = app
        super().__init__()
        self.aggregator = aggregator
        self.stop_evt = stop_evt
        pg.setConfigOptions(useOpenGL=True, antialias=False)

        self.setWindowTitle("Live laser height map (top view)")
        self.resize(1200, 800)

        self.layout = pg.GraphicsLayoutWidget(show=False)
        self.setCentralWidget(self.layout)

        self.plot = self.layout.addPlot(row=0, col=0)
        self.plot.setLabel("bottom", "Gantry X", units="mm")
        self.plot.setLabel("left", "Gantry Y", units="mm")
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setAspectLocked(True, ratio=1)

        self.scatter = pg.ScatterPlotItem(pen=None, size=SCATTER_DIAMETER_PX, pxMode=True)
        self.plot.addItem(self.scatter)

        self.legend = pg.LabelItem(justify="left")
        self.layout.addItem(self.legend, row=1, col=0)

        try:
            self.cmap = pg.colormap.get("viridis")
        except AttributeError:  # pragma: no cover - fallback for older pyqtgraph
            self.cmap = pg.ColorMap(
                pos=np.linspace(0.0, 1.0, num=4, dtype=float),
                color=["#440154", "#31688e", "#35b779", "#fde725"],
            )
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._refresh_plot)

    def start(self):
        self.show()
        self.timer.start(PLOT_REFRESH_MS)
        try:
            self.app.exec()
        finally:
            self.timer.stop()

    def closeEvent(self, event):  # noqa: N802 - Qt signature
        self.stop_evt.set()
        self.timer.stop()
        QtCore.QTimer.singleShot(0, self.app.quit)
        return super().closeEvent(event)

    def _refresh_plot(self):
        if self.stop_evt.is_set():
            if self.isVisible():
                self.close()
            return
        if not self.aggregator.has_fresh_data():
            return
        x_vals, y_vals, h_vals = self.aggregator.snapshot(max_points=DISPLAY_POINT_BUDGET)
        if x_vals.size == 0:
            self.scatter.clear()
            self.legend.setText("Awaiting data…")
            return

        finite = np.isfinite(h_vals)
        if not finite.any():
            return

        h_valid = h_vals[finite]
        vmin = float(np.nanmin(h_valid))
        vmax = float(np.nanmax(h_valid))
        span = max(vmax - vmin, 1e-6)
        norm = np.clip((h_vals - vmin) / span, 0.0, 1.0)
        if hasattr(np, "nan_to_num"):
            norm = np.nan_to_num(norm, nan=0.0, posinf=1.0, neginf=0.0)
        colors = self.cmap.map(norm, mode="qcolor")

        self.scatter.setData(x=x_vals, y=y_vals, brush=colors)
        self.plot.setXRange(float(np.min(x_vals) - AXIS_PADDING_MM), float(np.max(x_vals) + AXIS_PADDING_MM), padding=0)
        self.plot.setYRange(float(np.min(y_vals) - AXIS_PADDING_MM), float(np.max(y_vals) + AXIS_PADDING_MM), padding=0)

        self.legend.setText(
            f"Points shown: {x_vals.size:,} / stored {self.aggregator.point_count():,} | "
            f"Height span: {vmin:.1f} … {vmax:.1f} mm",
            size="12pt",
        )


class GantryLaserMapper:
    def __init__(self):
        self.scanner = LaserScanner()
        self.gantry = GantryJogController()
        self.aggregator = PointRingBuffer()
        self.stop_evt = threading.Event()
        self.profile_thread: Optional[threading.Thread] = None
        self._last_status_print = 0.0

    def run(self):
        # Ctrl+C friendly
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.scanner.connect()
        self.scanner.configure()
        self.gantry.start()

        self.profile_thread = threading.Thread(target=self._profile_loop, daemon=True)
        self.profile_thread.start()

        plotter = LiveHeightMapPlotter(self.aggregator, self.stop_evt)
        try:
            plotter.start()
        finally:
            self.stop_evt.set()
            self._cleanup()

    def _profile_loop(self):
        if not self.scanner.hLLT:
            return
        with transfer_profiles_guard(self.scanner.hLLT):
            while not self.stop_evt.is_set():
                if self.gantry.stop_evt.is_set():
                    self.stop_evt.set()
                    break
                pose = self.gantry.get_pose()
                if pose is None:
                    time.sleep(0.01)
                    continue
                data = self.scanner.grab_profile()
                if data is None:
                    continue
                x_np, z_np, i_np, lost = data
                if lost:
                    print(f"⚠️ Lost profiles: {lost}")
                valid = np.isfinite(z_np)
                if not valid.any():
                    continue
                x_line = x_np[valid] * LASER_LINE_FLIP_X + LASER_LINE_OFFSET_X_MM
                y_line = np.full_like(x_line, pose.y_mm + LASER_LINE_OFFSET_Y_MM)
                laser_height = self._laser_height_from_pose(pose)
                heights = laser_height - z_np[valid]
                x_world = pose.x_mm + x_line
                added = self.aggregator.add_points(x_world, y_line, heights)
                now = time.time()
                if added and now - self._last_status_print > 1.0:
                    self._last_status_print = now
                    print(
                        f"Points: {self.aggregator.point_count()} total | "
                        f"Pose X={pose.x_mm:.1f} mm, Y={pose.y_mm:.1f} mm"
                    )

    def _laser_height_from_pose(self, pose: PoseMM) -> float:
        axis_value = {
            "X": pose.x_mm,
            "Y": pose.y_mm,
            "Z": pose.z_mm,
        }[LASER_HEIGHT_AXIS.upper()]
        return LASER_HEIGHT_SCALE * axis_value + LASER_HEIGHT_OFFSET_MM

    def _cleanup(self):
        self.gantry.stop()
        if self.profile_thread:
            self.profile_thread.join(timeout=2.0)
        self.scanner.disconnect()
        print("Mapper shut down.")


def main():
    mapper = GantryLaserMapper()
    mapper.run()


if __name__ == "__main__":
    main()
