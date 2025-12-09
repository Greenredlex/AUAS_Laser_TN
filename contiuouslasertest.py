# run_scancontrol_live.py
# Live streaming of MICRO-EPSILON scanCONTROL profiles with Python.
# Requires: pyllt (Python bindings to LLT.dll), matplotlib, numpy

import sys
import time
import ctypes as ct
import numpy as np
import scipy as sp
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
    def __init__(self, hLLT, scanner_type, resolution, mode="single"):
        self.hLLT = hLLT
        self.scanner_type = scanner_type
        self.resolution = int(resolution)
        self.mode = mode  # "single" or "waterfall"

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
            # Increase font sizes / linewidths for PPT export
            plt.rcParams.update({
            "font.size": 18,
            "axes.labelsize": 20,
            "axes.titlesize": 22,
            "xtick.labelsize": 18,
            "ytick.labelsize": 18,
            "legend.fontsize": 18,
            "lines.linewidth": 2,
            })

            # Single realtime plot: x vs z with a full-range y-axis (no calibration)
            self.fig, self.ax_single = plt.subplots(figsize=(10, 6), facecolor="white")
            self.ax_single.grid(True)
            self.ax_single.set_xlabel("x (mm)", fontsize=20)
            self.ax_single.set_ylabel("z (mm)", fontsize=20)
            (self.line_single,) = self.ax_single.plot([], [], "g-", lw=2)
            # Max and min lines (thicker for visibility)
            (self.line_max,) = self.ax_single.plot([], [], "r--", lw=2, label="max")
            (self.line_min,) = self.ax_single.plot([], [], "b--", lw=2, label="min")
            self.ax_single.legend(loc="upper right", fontsize=18)
            # Ensure tick labels show plain mm without scientific offsets
            for axis in (self.ax_single.xaxis, self.ax_single.yaxis):
                fmt = ScalarFormatter(useOffset=False)
                fmt.set_scientific(False)
                axis.set_major_formatter(fmt)
            # Larger tick label rendering
            self.ax_single.tick_params(axis="both", which="major", labelsize=18)
            # Track global min/max encountered to avoid jittery autoscale
            self.global_z_min = np.inf
            self.global_z_max = -np.inf
            self.global_x_min = np.inf
            self.global_x_max = -np.inf
            # Rotation angle in degrees
            self.rotation_angle = 0.0

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
            raise ValueError("mode must be 'single' or 'waterfall'")

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

        if self.mode == "waterfall":
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
            self.line_max.set_data([], [])
            self.line_min.set_data([], [])
            return [self.line_single, self.line_max, self.line_min]
        else:
            self.im.set_data(np.zeros((self.rows, self.resolution), dtype=np.float32))
            return [self.im]

    def rotate_points(self, x, z, angle_deg):
        """Rotate points around their center by angle_deg (in degrees)."""
        if x.size == 0 or z.size == 0:
            return x, z
        # Center of points
        x_center = np.mean(x)
        z_center = np.mean(z)
        # Convert to radians and apply rotation matrix
        angle_rad = np.radians(angle_deg)
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)
        # Translate to origin
        x_trans = x - x_center
        z_trans = z - z_center
        # Rotate
        x_rot = x_trans * cos_a - z_trans * sin_a
        z_rot = x_trans * sin_a + z_trans * cos_a
        # Translate back
        x_rotated = x_rot + x_center
        z_rotated = z_rot + z_center
        return x_rotated, z_rotated

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

            # Apply rotation if angle is non-zero
            if self.rotation_angle != 0.0:
                xv, zv = self.rotate_points(xv, zv, self.rotation_angle)

            # Default to raw values; attempt smoothing only when sensible
            zv_spl = zv
            if xv.size > 1:
                # Ensure x is strictly ascending for spline: sort by x
                order = np.argsort(xv)
                xv_sorted = xv[order]
                zv_sorted = zv[order]

                # Handle duplicate x by averaging z values for the same x
                uniq_x, inv_idx = np.unique(xv_sorted, return_inverse=True)
                if uniq_x.size >= 2:
                    if uniq_x.size != xv_sorted.size:
                        zv_uniq = np.zeros(uniq_x.size, dtype=zv_sorted.dtype)
                        counts = np.zeros(uniq_x.size, dtype=np.int64)
                        for i, ui in enumerate(inv_idx):
                            zv_uniq[ui] += zv_sorted[i]
                            counts[ui] += 1
                        zv_uniq /= counts
                        xv_for_spline = uniq_x
                        zv_for_spline = zv_uniq
                    else:
                        xv_for_spline = xv_sorted
                        zv_for_spline = zv_sorted

                    try:
                        spline = sp.interpolate.make_smoothing_spline(xv_for_spline, zv_for_spline, lam=100)
                        # evaluate spline on the sorted x values, then map back to original order
                        zv_spl_sorted = spline(xv_sorted)
                        zv_spl = np.empty_like(zv)
                        zv_spl[order] = zv_spl_sorted
                    except Exception:
                        # Spline failed for some reason; keep raw values
                        zv_spl = zv
                else:
                    # Not enough unique x values for a spline
                    zv_spl = zv

            self.line_single.set_data(xv, zv_spl)

            # Calculate and draw max/min horizontal lines for this frame
            if zv_spl.size > 0:
                z_max = float(np.nanmax(zv_spl))
                z_min = float(np.nanmin(zv_spl))
                x_range = [xv.min(), xv.max()]
                self.line_max.set_data(x_range, [z_max, z_max])
                self.line_min.set_data(x_range, [z_min, z_min])

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
            return [self.line_single, self.line_max, self.line_min]
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

            # Keyboard handler for rotation control
            def on_key(event):
                if event.key == 'left':
                    # Rotate left (counter-clockwise) by 0.1 degrees
                    live.rotation_angle -= 0.1
                    print(f"Rotation angle: {live.rotation_angle:.1f}°")
                elif event.key == 'right':
                    # Rotate right (clockwise) by 0.1 degrees
                    live.rotation_angle += 0.1
                    print(f"Rotation angle: {live.rotation_angle:.1f}°")

            live.fig.canvas.mpl_connect('key_press_event', on_key)
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