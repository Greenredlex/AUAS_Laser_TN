import ctypes as ct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Helpers from your notebook
# - hLLT connected and configured
# - scanner_type (ct.c_int), resolution (ct.c_uint), etc.

# Buffers once, reused each frame
resolution_val = int(resolution)
profile_buffer = (ct.c_ubyte * (resolution_val * 64))()
lost_profiles = ct.c_int()

x = (ct.c_double * resolution_val)()
z = (ct.c_double * resolution_val)()
intens = (ct.c_ushort * resolution_val)()

snull_us = ct.POINTER(ct.c_ushort)()
snull_ui = ct.POINTER(ct.c_uint)()

# Start continuous transfer once
ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
if ret < 1:
    raise RuntimeError(f"transfer_profiles start failed: {ret}")

# Matplotlib live plot setup
plt.figure(figsize=(7, 6), facecolor="white")
ax1 = plt.subplot(211)
ax1.grid(True)
ax1.set_ylabel("z")
line_profile, = ax1.plot([], [], "g.", ms=2)
ax1.set_xlim(-30, 30)      # adjust to your sensor's FoV
ax1.set_ylim(25, 135)

ax2 = plt.subplot(212)
ax2.grid(True)
ax2.set_xlabel("x")
ax2.set_ylabel("intensity")
line_int, = ax2.plot([], [], "r.", ms=2)
ax2.set_xlim(-60, 60)
ax2.set_ylim(0, 1200)

# Optionally, precompute x per resolution if your x does not change between frames.
# For many modes x is static; if so, compute once and reuse to save time.
static_x = None

def grab_and_convert():
    # Try to fetch latest profile (non-blocking).
    # If no new profile, LLT returns >0 size anyway; you can add small sleep if needed.
    r = llt.get_actual_profile(
        hLLT,
        profile_buffer,
        len(profile_buffer),
        llt.TProfileConfig.PROFILE,
        ct.byref(lost_profiles),
    )
    if r < 1:
        return None

    # Convert to values (x, z, intensity)
    # flags: 0=scaled? 1=reflection? -> follow your working call
    r2 = llt.convert_profile_2_values(
        hLLT,
        profile_buffer,
        resolution_val,
        llt.TProfileConfig.PROFILE,
        scanner_type,
        0,   # null points to 0
        1,   # scale/convert mode as in your code
        snull_us,         # saturation
        intens,           # intensities
        snull_us,         # width
        x, z,             # x, z
        snull_ui, snull_ui
    )
    if r2 < 1:
        return None

    # Convert to numpy for plotting
    x_np = np.frombuffer(x, dtype=np.float64, count=resolution_val)
    z_np = np.frombuffer(z, dtype=np.float64, count=resolution_val)
    i_np = np.frombuffer(intens, dtype=np.uint16, count=resolution_val)
    return x_np, z_np, i_np, lost_profiles.value

def init_anim():
    line_profile.set_data([], [])
    line_int.set_data([], [])
    return line_profile, line_int

def update(_frame):
    data = grab_and_convert()
    if data is None:
        return line_profile, line_int
    x_np, z_np, i_np, lost = data

    # Optional: mask invalid points (NaN or zero intensity)
    valid = np.isfinite(z_np)
    # Example: also drop zero intensity
    valid &= (i_np > 0)

    xv = x_np[valid]
    zv = z_np[valid]
    iv = i_np[valid]

    line_profile.set_data(xv, zv)
    line_int.set_data(xv, iv)

    # If your scene range changes, you can auto-scale:
    # ax1.relim(); ax1.autoscale_view()
    # ax2.relim(); ax2.autoscale_view()

    return line_profile, line_int

ani = FuncAnimation(
    plt.gcf(),
    update,
    init_func=init_anim,
    interval=30,        # ms between redraws; adjust to your desired UI rate
    blit=True
)

try:
    plt.show()
finally:
    # Always stop transfer when leaving
    llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)