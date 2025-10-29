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
# sys.path.append(r"C:\path\to\python_bindings")

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

    # Choose the highest or simply first entry (as in your notebook)
    resolution = int(available_resolutions[0])
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
        self.mode = mode  # "scatter" or "waterfall"

        # Buffers
        self.profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        self.lost_profiles = ct.c_int(0)

        self.x = (ct.c_double * self.resolution)()
        self.z = (ct.c_double * self.resolution)()
        self.i = (ct.c_ushort * self.resolution)()

        self.snull_us = ct.POINTER(ct.c_ushort)()
        self.snull_ui = ct.POINTER(ct.c_uint)()

        # Matplotlib
        if self.mode == "scatter":
            self.fig = plt.figure(figsize=(7, 6), facecolor="white")
            self.ax1 = self.fig.add_subplot(211)
            self.ax2 = self.fig.add_subplot(212)

            self.ax1.grid(True)
            self.ax2.grid(True)
            self.ax1.set_ylabel("z")
            self.ax2.set_xlabel("x")
            self.ax2.set_ylabel("intensity")

            # Set to typical values; adjust to your scene
            self.ax1.set_xlim(-30, 30)
            self.ax1.set_ylim(25, 135)
            self.ax2.set_xlim(-60, 60)
            self.ax2.set_ylim(0, 1200)

            (self.line_profile,) = self.ax1.plot([], [], "g.", ms=2)
            (self.line_int,) = self.ax2.plot([], [], "r.", ms=2)

        elif self.mode == "waterfall":
            from collections import deque

            self.rows = 200
            self.deque = deque(maxlen=self.rows)
            self.fig, self.ax = plt.subplots(figsize=(7, 4), facecolor="white")
            self.im = self.ax.imshow(
                np.zeros((self.rows, self.resolution), dtype=np.float32),
                vmin=0,
                vmax=1200,
                aspect="auto",
                cmap="viridis",
                origin="lower",
            )
            self.ax.set_xlabel("x index")
            self.ax.set_ylabel("recent profiles")
            self.ax.set_title("Intensity waterfall")
            self.fig.colorbar(self.im, ax=self.ax)
        else:
            raise ValueError("mode must be 'scatter' or 'waterfall'")

        self._frames_seen = 0

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

        x_np = np.frombuffer(self.x, dtype=np.float64, count=self.resolution)
        z_np = np.frombuffer(self.z, dtype=np.float64, count=self.resolution)
        i_np = np.frombuffer(self.i, dtype=np.uint16, count=self.resolution)

        return x_np, z_np, i_np, self.lost_profiles.value

    def init(self):
        if self.mode == "scatter":
            self.line_profile.set_data([], [])
            self.line_int.set_data([], [])
            return self.line_profile, self.line_int
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

        if self.mode == "scatter":
            valid = np.isfinite(z_np)
            valid &= i_np > 0
            xv = x_np[valid]
            zv = z_np[valid]
            iv = i_np[valid]
            self.line_profile.set_data(xv, zv)
            self.line_int.set_data(xv, iv)
            return self.line_profile, self.line_int
        else:
            # waterfall (intensity)
            self.deque.append(i_np.astype(np.float32))
            arr = np.vstack(self.deque)
            if arr.shape[0] < self.rows:
                pad = np.zeros((self.rows - arr.shape[0], arr.shape[1]), dtype=arr.dtype)
                arr = np.vstack([pad, arr])
            self.im.set_data(arr)
            return [self.im]


def run(mode="scatter", interval_ms=30, timeout_first_frame=5.0):
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
    # Choose mode: "scatter" for x/z + intensity, or "waterfall" for 2D intensity image
    run(mode="scatter", interval_ms=30)