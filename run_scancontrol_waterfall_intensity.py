# PyQtGraph waterfall for MICRO-EPSILON scanCONTROL (intensity)
# Requires: pyllt (DLL bindings), PyQt5 or PySide6, pyqtgraph, numpy

import sys
import time
import ctypes as ct
import numpy as np
from threading import Thread, Event
from collections import deque

# If needed, add bindings path
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector\libs\python_bindings")

try:
    import pyllt as llt
except ImportError:
    from pyllt import pyllt as llt  # type: ignore

# Choose Qt binding for pyqtgraph
try:
    from PyQt5 import QtWidgets
except Exception:
    from PySide6 import QtWidgets  # fallback

import pyqtgraph as pg


def check_ok(code, name):
    if code >= 1:
        print(f"{name}: OK ({code})")
        return True
    print(f"{name}: FAIL ({code})")
    return False


def ip_int_to_str(ip_int: int) -> str:
    return f"{ip_int & 0xFF}.{(ip_int >> 8) & 0xFF}.{(ip_int >> 16) & 0xFF}.{(ip_int >> 24) & 0xFF}"


def connect_scanner():
    hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
    if not hLLT:
        raise RuntimeError("create_llt_device failed")

    interfaces = (ct.c_uint * 6)()
    r = llt.get_device_interfaces_fast(hLLT, interfaces, len(interfaces))
    if not check_ok(r, "get_device_interfaces_fast"):
        raise RuntimeError("No interfaces found")

    detected = interfaces[0]
    if detected == 0:
        raise RuntimeError("No scanner found")
    print("Scanner:", ip_int_to_str(detected))

    r = llt.set_device_interface(hLLT, detected, 0)
    if not check_ok(r, "set_device_interface"):
        raise RuntimeError("set_device_interface failed")

    r = llt.connect(hLLT)
    if not check_ok(r, "connect"):
        raise RuntimeError(f"connect failed: {r}")

    return hLLT


def setup_scanner(hLLT):
    scanner_type = ct.c_int(0)
    r = llt.get_llt_type(hLLT, ct.byref(scanner_type))
    if not check_ok(r, "get_llt_type"):
        raise RuntimeError("get_llt_type failed")

    av = (ct.c_uint * 4)()
    r = llt.get_resolutions(hLLT, av, len(av))
    if not check_ok(r, "get_resolutions"):
        raise RuntimeError("get_resolutions failed")
    resolution = int(av[0])
    r = llt.set_resolution(hLLT, resolution)
    if not check_ok(r, "set_resolution"):
        raise RuntimeError("set_resolution failed")

    r = llt.set_profile_config(hLLT, llt.TProfileConfig.PROFILE)
    if not check_ok(r, "set_profile_config(PROFILE)"):
        raise RuntimeError("set_profile_config failed")

    return scanner_type, resolution


class AcquisitionThread(Thread):
    def __init__(self, hLLT, scanner_type, resolution, rows=300, poll_sleep=0.001):
        super().__init__(daemon=True)
        self.hLLT = hLLT
        self.scanner_type = scanner_type
        self.resolution = int(resolution)
        self.rows = rows
        self.poll_sleep = poll_sleep

        self.stop_evt = Event()
        self.started_transfer = False

        # Buffers
        self.profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        self.lost_profiles = ct.c_int(0)
        self.x = (ct.c_double * self.resolution)()
        self.z = (ct.c_double * self.resolution)()
        self.i = (ct.c_ushort * self.resolution)()
        self.snull_us = ct.POINTER(ct.c_ushort)()
        self.snull_ui = ct.POINTER(ct.c_uint)()

        self.deque = deque(maxlen=self.rows)  # intensity rows

    def start_transfer(self):
        if not self.started_transfer:
            r = llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
            if r < 1:
                raise RuntimeError(f"transfer_profiles start failed: {r}")
            self.started_transfer = True

    def stop_transfer(self):
        if self.started_transfer:
            try:
                llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
            except Exception:
                pass
            self.started_transfer = False

    def grab(self):
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
            return False

        intens = np.frombuffer(self.i, dtype=np.uint16, count=self.resolution).astype(np.float32)
        self.deque.append(intens)
        return True

    def run(self):
        try:
            self.start_transfer()

            # prime for first frame
            t0 = time.time()
            while not self.stop_evt.is_set():
                if self.grab():
                    break
                if time.time() - t0 > 5.0:
                    print("Timeout waiting first profile")
                    return

            # main loop
            while not self.stop_evt.is_set():
                self.grab()
                time.sleep(self.poll_sleep)
        finally:
            self.stop_transfer()

    def latest_image(self):
        if len(self.deque) == 0:
            return None
        arr = np.vstack(self.deque)
        if arr.shape[0] < self.rows:
            pad = np.zeros((self.rows - arr.shape[0], arr.shape[1]), dtype=np.float32)
            arr = np.vstack([pad, arr])
        return arr


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, hLLT, scanner_type, resolution):
        super().__init__()
        self.setWindowTitle("scanCONTROL Waterfall (Intensity)")
        self.resize(900, 500)

        self.view = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.view)
        self.plot = self.view.addPlot()
        self.plot.setLabel("left", "recent profiles")
        self.plot.setLabel("bottom", "x index")
        self.img = pg.ImageItem()
        self.plot.addItem(self.img)
        self.plot.setAspectLocked(False)
        self.plot.invertY(True)

        self.thread = AcquisitionThread(hLLT, scanner_type, resolution, rows=300)
        self.thread.start()

        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(30)

    def on_timer(self):
        arr = self.thread.latest_image()
        if arr is None:
            return
        # Scale display range for intensity; adjust as needed
        self.img.setImage(arr, autoLevels=False)
        self.img.setLevels((0, 1200))

    def closeEvent(self, ev):
        self.timer.stop()
        self.thread.stop_evt.set()
        self.thread.join(timeout=2.0)
        super().closeEvent(ev)


def main():
    app = QtWidgets.QApplication(sys.argv)
    hLLT = None
    try:
        hLLT = connect_scanner()
        scanner_type, resolution = setup_scanner(hLLT)
        win = MainWindow(hLLT, scanner_type, resolution)
        win.show()
        app.exec_()
    finally:
        if hLLT:
            try:
                llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
            except Exception:
                pass
            try:
                llt.disconnect(hLLT)
            except Exception:
                pass


if __name__ == "__main__":
    main()