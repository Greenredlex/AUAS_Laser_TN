# run_scancontrol_live.py
# Live streaming of MICRO-EPSILON scanCONTROL profiles with Python.
# Requires: pyllt (Python bindings to LLT.dll), matplotlib, numpy

import sys
import time
import ctypes as ct
import numpy as np
import signal
from contextlib import contextmanager

# Optional: add your python_bindings folder to sys.path if pyllt is not installed
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector\libs\python_bindings")

try:
    import pyllt as llt
except ImportError:
    # Some packages expose as from pyllt import pyllt as llt
    try:
        from pyllt import pyllt as llt  # type: ignore
    except Exception as e:
        print("ERROR: Could not import pyllt. Adjust sys.path to point to python_bindings.")
        raise

import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
from matplotlib.animation import FuncAnimation


# ------------------------
# Utility / helpers
# ------------------------
def check_llt_success(result: int, operation_name: str) -> bool:
    # For the LLT API: result >= 1 = success, < 1 = error
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


# ------------------------
# Core setup
# ------------------------
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
    # Per vendor examples: third parameter set to 0
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

    # Scanner type
    scanner_type = ct.c_int(0)
    res = llt.get_llt_type(hLLT, ct.byref(scanner_type))
    if not check_llt_success(res, "get_llt_type"):
        raise RuntimeError("get_llt_type failed")
    print(f"Scanner type: {scanner_type.value}")

    # Resolutions
    available_resolutions = (ct.c_uint * 4)()
    res = llt.get_resolutions(hLLT, available_resolutions, len(available_resolutions))
    if not check_llt_success(res, "get_resolutions"):
        raise RuntimeError("get_resolutions failed")

    # Choose the highest available non-zero resolution for better fidelity
    res_list = [int(v) for v in available_resolutions if int(v) > 0]
    resolution = max(res_list) if res_list else int(available_resolutions[0])
    print(f"Using resolution: {resolution}")
    res = llt.set_resolution(hLLT, resolution)
    if not check_llt_success(res, "set_resolution"):
        raise RuntimeError("set_resolution failed")

    # Profile config: standard profile
    res = llt.set_profile_config(hLLT, llt.TProfileConfig.PROFILE)
    if not check_llt_success(res, "set_profile_config(PROFILE)"):
        raise RuntimeError("set_profile_config failed")

    return scanner_type, resolution


# ------------------------
# Acquisition/plotting
# ------------------------
class LivePlot:
    def __init__(self, hLLT, scanner_type, resolution, mode="scatter"):
        self.hLLT = hLLT
        self.scanner_type = scanner_type
        self.resolution = int(resolution)
        self.mode = mode  # "single", "scatter" or "waterfall"

        # Buffers
        self.profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        self.lost_profiles = ct.c_int(0)

        self.x = (ct.c_double * self.resolution)()
        self.z = (ct.c_double * self.resolution)()
        self.i = (ct.c_ushort * self.resolution)()

        self.snull_us = ct.POINTER(ct.c_ushort)()
        self.snull_ui = ct.POINTER(ct.c_uint)()

        # Matplotlib
        if self.mode == "single":
            # Single realtime plot: x vs z with a full-range y-axis (no calibration)
            self.fig, self.ax_single = plt.subplots(figsize=(7, 5), facecolor="white")
            self.ax_single.grid(True)
            self.ax_single.set_xlabel("x (mm)")
            self.ax_single.set_ylabel("z (mm)")
            (self.line_single,) = self.ax_single.plot([], [], "g-", lw=1)
            # Ensure tick labels show plain mm without scientific offsets
            for axis in (self.ax_single.xaxis, self.ax_single.yaxis):
                fmt = ScalarFormatter(useOffset=False)
                fmt.set_scientific(False)
                axis.set_major_formatter(fmt)
            # Track global min/max encountered to avoid jittery autoscale
            self.global_z_min = np.inf
            self.global_z_max = -np.inf
            self.global_x_min = np.inf
            self.global_x_max = -np.inf
        elif self.mode == "scatter":
            self.fig = plt.figure(figsize=(7, 6), facecolor="white")
            self.ax1 = self.fig.add_subplot(211)
            self.ax2 = self.fig.add_subplot(212)

            self.ax1.grid(True)
            self.ax2.grid(True)
            self.ax1.set_ylabel("z")
            self.ax2.set_xlabel("x")
            # Show millimetres for the second subplot instead of intensity
            self.ax2.set_ylabel("z (mm)")

            # Set to typical values; adjust to your scene
            self.ax1.set_xlim(-30, 30)
            self.ax1.set_ylim(25, 135)
            # Make the second subplot show the same sensible z-range (mm)
            self.ax2.set_xlim(-60, 60)
            self.ax2.set_ylim(25, 135)

            (self.line_profile,) = self.ax1.plot([], [], "g.", ms=2)
            # Plot z (mm) on the second axes instead of intensity
            (self.line_z_mm,) = self.ax2.plot([], [], "r-", ms=2, lw=1)

        elif self.mode == "waterfall":
            from collections import deque

            self.rows = 200
            self.deque = deque(maxlen=self.rows)
            self.fig, self.ax = plt.subplots(figsize=(7, 4), facecolor="white")
            # Waterfall of z values (mm) instead of intensity
            self.im = self.ax.imshow(
                np.zeros((self.rows, self.resolution), dtype=np.float32),
                vmin=25,
                vmax=135,
                aspect="auto",
                cmap="viridis",
                origin="lower",
            )
            self.ax.set_xlabel("x index")
            self.ax.set_ylabel("recent profiles")
            self.ax.set_title("Z (mm) waterfall")
            cbar = self.fig.colorbar(self.im, ax=self.ax)
            cbar.set_label("z (mm)")
        else:
            raise ValueError("mode must be 'single', 'scatter' or 'waterfall'")

        self._frames_seen = 0
        # Dynamic range calibration placeholders (will be filled by calibrate_range)
        self.z_min_cal = None
        self.z_max_cal = None
        # Unit scaling: auto-detect meters vs millimetres on first valid frame
        self.unit_scale = None  # will be set to 1 (mm) or 1000 (m->mm)

    def calibrate_range(self, samples=30, sleep_s=0.005):
        """Grab a handful of profiles to establish a realistic z-range.
        We ignore zeros (null/out-of-range) and use robust percentiles so a tall
        spike doesn't destroy the viewing window. If calibration fails, we keep
        existing axis limits.
        """
        z_vals = []
        for _ in range(samples):
            data = self.grab()
            if not data:
                time.sleep(sleep_s)
                continue
            # Proper unpacking
            x_np, z_np, i_np, _lost = data
            # Filter: valid finite, intensity >0, and z > 0
            mask = np.isfinite(z_np) & (i_np > 0) & (z_np > 0)
            if mask.any():
                z_vals.append(z_np[mask])
            time.sleep(sleep_s)
        if not z_vals:
            return False
        z_all = np.concatenate(z_vals)
        if z_all.size < 5:
            return False
        z_lo = float(np.percentile(z_all, 5))
        z_hi = float(np.percentile(z_all, 95))
        # Expand a little margin (10%)
        span = z_hi - z_lo
        if span <= 0:
            return False
        z_lo -= span * 0.1
        z_hi += span * 0.1
        self.z_min_cal = z_lo
        self.z_max_cal = z_hi
        # Apply to existing axes
        if self.mode == "scatter":
            try:
                self.ax1.set_ylim(z_lo, z_hi)
                self.ax2.set_ylim(z_lo, z_hi)
            except Exception:
                pass
        elif self.mode == "waterfall":
            try:
                self.im.set_clim(z_lo, z_hi)
            except Exception:
                pass
        print(f"Calibrated z-range: [{z_lo:.2f}, {z_hi:.2f}] (approx span {z_hi - z_lo:.2f} mm)")
        return True

    def grab(self):
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
            0,  # null points set to 0
            1,  # convert/scale flag as in your notebook
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

        # Copy into writable arrays for post-processing
        x_np = np.frombuffer(self.x, dtype=np.float64, count=self.resolution).copy()
        z_np = np.frombuffer(self.z, dtype=np.float64, count=self.resolution).copy()
        i_np = np.frombuffer(self.i, dtype=np.uint16, count=self.resolution).copy()

        # Treat invalid data consistently: intensity=0 or z<=0 => NaN
        invalid = (i_np == 0) | (~np.isfinite(z_np)) | (z_np <= 0)
        if invalid.any():
            z_np[invalid] = np.nan

        # Auto-detect unit scale once: if values look like meters (< 5), convert to mm
        if self.unit_scale is None:
            valid = np.isfinite(z_np)
            if np.any(valid):
                p95_z = float(np.nanpercentile(np.abs(z_np[valid]), 95))
                # For x, use any finite sample (x has no NaNs)
                p95_x = float(np.nanpercentile(np.abs(x_np), 95)) if x_np.size else 0.0
                # Heuristic: if both are small (<5), assume meters and scale to mm
                self.unit_scale = 1000.0 if (p95_z < 5.0 and p95_x < 5.0) else 1.0
                print(f"Unit scale set to {self.unit_scale} (1=mm, 1000=m→mm). p95_x={p95_x:.3f}, p95_z={p95_z:.3f}")
            else:
                # No valid data yet; try again next frame
                self.unit_scale = None

        if self.unit_scale and self.unit_scale != 1.0:
            x_np *= self.unit_scale
            z_np *= self.unit_scale

        return x_np, z_np, i_np, self.lost_profiles.value

    def init(self):
        if self.mode == "single":
            self.line_single.set_data([], [])
            return [self.line_single]
        if self.mode == "scatter":
            self.line_profile.set_data([], [])
            self.line_z_mm.set_data([], [])
            return self.line_profile, self.line_z_mm
        else:
            self.im.set_data(np.zeros((self.rows, self.resolution), dtype=np.float32))
            return [self.im]

    def update(self, _):
        data = self.grab()
        if data is None:
            # No new profile. Let the animation continue.
            return self.init()

        x_np, z_np, i_np, lost = data
        self._frames_seen += 1
        if lost:
            print(f"⚠️ Lost profiles: {lost}")

        if self.mode == "single":
            # Valid finite z only; invalid already set to NaN in grab()
            valid = np.isfinite(z_np)
            xv = x_np[valid]
            zv = z_np[valid]
            self.line_single.set_data(xv, zv)
            # Expand global ranges and set limits with margin
            if xv.size and zv.size:
                self.global_x_min = min(self.global_x_min, float(np.nanmin(xv)))
                self.global_x_max = max(self.global_x_max, float(np.nanmax(xv)))
                self.global_z_min = min(self.global_z_min, float(np.nanmin(zv)))
                self.global_z_max = max(self.global_z_max, float(np.nanmax(zv)))
                # Add a small margin
                x_span = max(1e-6, self.global_x_max - self.global_x_min)
                z_span = max(1e-6, self.global_z_max - self.global_z_min)
                xm = x_span * 0.05
                zm = z_span * 0.05
                self.ax_single.set_xlim(self.global_x_min - xm, self.global_x_max + xm)
                self.ax_single.set_ylim(self.global_z_min - zm, self.global_z_max + zm)
            return [self.line_single]
        if self.mode == "scatter":
            # Validity: finite z, intensity >0, z>0; drop zeros (nulls) & outside calibrated span if available
            valid = np.isfinite(z_np)
            # If a calibration window exists, prefer it, but be ready to recalibrate if too many points fall outside
            if self.z_min_cal is not None and self.z_max_cal is not None:
                in_win = (z_np >= self.z_min_cal) & (z_np <= self.z_max_cal)
                # If more than 40% are outside, trigger recalibration (scene moved)
                frac_out = 1.0 - float(np.count_nonzero(in_win)) / max(1, z_np.size)
                if frac_out > 0.4 and self._frames_seen % 10 == 0:
                    self.calibrate_range(samples=15)
                valid &= in_win
            xv = x_np[valid]
            zv = z_np[valid]
            self.line_profile.set_data(xv, zv)
            self.line_z_mm.set_data(xv, zv)
            # If not calibrated yet, attempt once after a few frames
            if self.z_min_cal is None and self._frames_seen == 10:
                self.calibrate_range()
            return self.line_profile, self.line_z_mm
        else:
            # waterfall (z in mm)
            # Replace invalid or out-of-range with NaN for better colour scaling
            valid = np.isfinite(z_np) & (z_np > 0)
            z_clean = z_np.copy()
            if self.z_min_cal is not None and self.z_max_cal is not None:
                valid &= (z_clean >= self.z_min_cal) & (z_clean <= self.z_max_cal)
            z_clean[~valid] = np.nan
            self.deque.append(z_clean.astype(np.float32))
            arr = np.vstack(self.deque)
            if arr.shape[0] < self.rows:
                pad = np.zeros((self.rows - arr.shape[0], arr.shape[1]), dtype=arr.dtype)
                arr = np.vstack([pad, arr])
            self.im.set_data(arr)
            return [self.im]


def run(mode="single", interval_ms=30, timeout_first_frame=5.0):
    # Ctrl+C friendly
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    hLLT = None
    try:
        hLLT, _ = connect_scanner()
        scanner_type, resolution = setup_scanner(hLLT)

        live = LivePlot(hLLT, scanner_type, resolution, mode=mode)

        # Start streaming
        with safe_transfer(hLLT):
            # Warm up: try to get at least one profile before showing the window
            t0 = time.time()
            first_ok = False
            while time.time() - t0 < timeout_first_frame:
                if live.grab() is not None:
                    first_ok = True
                    break
                time.sleep(0.01)

            if not first_ok:
                print("ERROR: No profiles received within timeout. Check network/sensor.")
                return

            # Attempt an early calibration only for modes that use it
            if mode != "single":
                live.calibrate_range(samples=20)

            ani = FuncAnimation(
                live.fig,
                live.update,
                init_func=live.init,
                interval=interval_ms,
                blit=True,
            )
            plt.show()

    finally:
        try:
            if hLLT is not None:
                # Ensure transfer stopped by context manager, then disconnect
                llt.disconnect(hLLT)
                print("Disconnected.")
        except Exception:
            pass


if __name__ == "__main__":
    # Default: single realtime x–z plot with full-range y-axis (no calibration)
    run(mode="single", interval_ms=30)