<<<<<<< HEAD
# AUAS laser gantry mapper

This workspace now contains `gantry_scan_mapper.py`, a combined script that lets you
jog the CRI gantry with the familiar **WASDQE** keys while continuously capturing
scanCONTROL line profiles and building a live 2D top-down height map.

## Controls & workflow

1. Make sure the CRI controller and the MICRO-EPSILON scanner are both powered and reachable.
2. Install the required Python packages (run from the repo root):

```powershell
pip install keyboard matplotlib numpy
```

> ℹ️ The `keyboard` package requires Administrator mode on Windows to capture global key presses.

3. Ensure the CRI and `pyllt` libraries from `scenario_inspector` stay in the hard-coded
   paths used by the existing scripts (adjust the `sys.path.append` lines otherwise).
4. Run the combined mapper:

```powershell
python gantry_scan_mapper.py
```

5. Hold **W/S** for gantry axis A1, **A/D** for A2, **Q/E** for A3. Use **Space** to zero all speeds
   and **Esc** to quit. As you sweep the gantry, the figure window will keep adding the latest
   laser line to the top-view heat map (X axis = gantry X, Y axis = gantry Y, colour = height in mm).

## Height fusion / calibration

The mapper converts each scanCONTROL profile into absolute surface heights via:

$$h_{object} = h_{laser}(\text{cardpos}) - d_{laser}(\text{profile})$$

Because the CRI "cardpos" height reportedly increases in the opposite direction, the script
applies an inversion factor (`LASER_HEIGHT_SCALE = -1`). If your rig uses a different axis or
sign convention, adjust the configuration block near the top of `gantry_scan_mapper.py`:

- `POSE_OFFSETS_M`: shifts the CRI pose so `(0,0,0)` roughly matches your table origin.
- `LASER_LINE_FLIP_X` / `LASER_LINE_OFFSET_X_MM`: align the scan line with gantry +X.
- `LASER_LINE_OFFSET_Y_MM`: lateral offset if the laser is not perfectly above axis A2.
- `LASER_HEIGHT_AXIS`, `LASER_HEIGHT_SCALE`, `LASER_HEIGHT_OFFSET_MM`: choose which gantry axis
  tracks the physical laser height and how to translate it to mm.

Tune these values once (for example by parking the gantry at a known reference block) and the
height map will remain consistent even when you jog the laser up/down while scanning.

## Output & next steps

- The scatter plot keeps up to 400k points in memory; reduce `MAX_POINTS_IN_MAP` if RAM is tight.
- Saving a snapshot is as simple as clicking the Matplotlib save icon.
- For automated raster scans you can replace the manual jogger with waypoints, but the laser
  and plotting infrastructure in `gantry_scan_mapper.py` stays the same.

## Xbox controller jog mode

If you prefer an analog gamepad over the keyboard jogger, `gantrycontrollerbasic.py` now exposes
an Xbox control loop:

1. Install the additional dependency:

```powershell
pip install pygame
```

2. Connect a USB Xbox controller (tested with the Xbox One pad) and run:

```powershell
python gantrycontrollerbasic.py --mode xbox
```

3. Controls:
   - **Left stick** &rarr; Gantry axes A1 (X) and A2 (Y)
   - **Right stick (vertical)** &rarr; Gantry axis A3 (Z)
   - **Right stick (horizontal)** &rarr; Velocity ramp from 0% (far left) to 100% (far right)
   - **B button** &rarr; Stop jogging and exit

You can still trigger absolute test moves via:

```powershell
python gantrycontrollerbasic.py --mode move --xpos 100 --ypos 20 --zpos 30
```

Use `--max-speed` to clamp the jog speed if you need gentler motion, e.g. `--max-speed 40`.
=======
Active code base to control a LaserLine scanner on a 3 axis gantry. Using realtime calculations to create a 3d heigh map of a object
>>>>>>> 4af72cb18ba5113a6597ad35ebdd244816be7e08
