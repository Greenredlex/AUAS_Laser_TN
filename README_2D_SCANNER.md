# Real-time 2D Laser Scanner with Gantry Compensation

This project provides real-time 2D visualization of laser scanner data combined with gantry position for height-compensated scanning.

## Overview

The system combines:
- **Laser Scanner**: Provides a line of height data along the X-axis
- **Gantry Controller**: 3-axis system that moves the laser scanner
- **Real-time Visualization**: 2D plot with X (laser) on x-axis, Y (gantry) on y-axis, and color representing height

### Key Feature: Height Compensation
The system automatically subtracts the gantry's Z position from the laser height measurements, allowing you to move the laser up and down while maintaining accurate object height measurements.

## Files

### 1. `realtime_2d_scanner.py` (Full-featured version)
- Uses PyQtGraph for high-performance visualization
- Real-time scatter plot with dynamic color mapping
- Interactive controls (pause, clear data)
- Thread-safe data acquisition
- Best for production use and high frame rates

**Features:**
- Thread-safe laser and gantry data acquisition
- Dynamic colorbar based on height percentiles
- Up to 10,000 scan points in memory
- ~50 Hz visualization update rate
- FPS counter and point counter
- Pause/resume scanning
- Clear data button

### 2. `realtime_2d_scanner_simple.py` (Simplified version)
- Uses Matplotlib for simpler deployment
- Easier to understand and modify
- No PyQt5/PySide6 dependency for visualization
- Good for testing and learning

**Features:**
- Combined data acquisition class
- Matplotlib animation
- Auto-scaling axes and color limits
- Up to 5,000 scan points in memory
- ~10 Hz visualization update rate

## Configuration

Both scripts have configuration constants at the top that you can adjust:

```python
# Gantry connection
GANTRY_IP = "192.168.3.11"
GANTRY_PORT = 3920

# Gantry offset calibration (IMPORTANT!)
GANTRY_X_OFFSET = -0.9  # offset to convert robot X to world X
GANTRY_Y_OFFSET = -1.0  # offset to convert robot Y to world Y
GANTRY_Z_OFFSET = 1.3   # offset to convert robot Z to world Z

# Performance settings
MAX_SCAN_POINTS = 10000  # Maximum points stored in memory
UPDATE_RATE_MS = 50      # GUI update rate (milliseconds)
```

### Calibrating Gantry Offsets

The offsets are used to convert robot coordinates to world coordinates:
```
world_x = robot.position.X + GANTRY_X_OFFSET
world_y = robot.position.Y + GANTRY_Y_OFFSET
world_z = robot.position.Z + GANTRY_Z_OFFSET
```

To calibrate:
1. Move the gantry to a known reference position
2. Compare the robot coordinates with your desired world coordinates
3. Calculate the offsets needed
4. Update the constants in the script

## Installation

### Required Python Packages

```bash
pip install numpy
pip install matplotlib
pip install keyboard
pip install pyqtgraph  # For realtime_2d_scanner.py
pip install PyQt5      # OR PySide6 (for realtime_2d_scanner.py)
```

### Required Hardware
- Micro-Epsilon scanCONTROL laser scanner (connected via Ethernet)
- CRI/CRCL-compatible gantry system (connected via network)

## Usage

### Running the Full-Featured Version

```bash
python realtime_2d_scanner.py
```

**Controls:**
- Move the gantry (using your gantry controller or manual jog) to scan
- Click "Clear Data" to reset the visualization
- Click "Pause" to pause data acquisition
- Close the window to exit

### Running the Simplified Version

```bash
python realtime_2d_scanner_simple.py
```

**Controls:**
- Move the gantry to scan
- Close the plot window to exit

## How It Works

### Data Flow

```
┌─────────────────┐
│ Laser Scanner   │  Provides: X positions, Z heights, Intensity
│  (Thread 1)     │            (line scan along X-axis)
└────────┬────────┘
         │
         ├──► Latest Profile (thread-safe)
         │
         ▼
┌─────────────────┐
│  Main App       │  Combines laser + gantry data
│  (Main Thread)  │  Compensates height: Z_obj = Z_laser - Z_gantry
└────────┬────────┘
         │
         ▲
         │
         ├──◄ Current Position (thread-safe)
         │
┌────────┴────────┐
│ Gantry Monitor  │  Provides: X, Y, Z position of gantry
│  (Thread 2)     │            (updated ~100 Hz)
└─────────────────┘
```

### Height Compensation Formula

```
Measured Object Height = Laser Z Reading - Gantry Z Position
```

This allows the gantry to move up and down while maintaining accurate object measurements.

### Coordinate System

- **X-axis**: Laser scanner coordinate (mm) - perpendicular to scan direction
- **Y-axis**: Gantry Y position (mm) - direction of travel
- **Color**: Compensated height (mm) - actual object height

## Visualization Details

### Color Mapping
- Uses "viridis" colormap by default
- Blue → Low height
- Green/Yellow → Medium height  
- Red → High height

### Dynamic Range
The color limits auto-adjust based on the 5th and 95th percentiles of the height data, preventing outliers from affecting the visualization.

### Point Filtering
Only points with:
- Intensity > 50
- Z > 0
- Finite (non-NaN) values
are included in the visualization.

## Troubleshooting

### "No laser scanner found"
- Check that the scanner is powered on and connected to the network
- Verify network settings and firewall
- Close any other software using the scanner

### "Failed to connect to gantry"
- Verify the IP address and port are correct
- Check network connectivity
- Ensure no other software is controlling the gantry

### Low frame rate
- Reduce `MAX_SCAN_POINTS` to decrease memory usage
- Increase `UPDATE_RATE_MS` to reduce update frequency
- Use the simple version if PyQtGraph performance is poor

### Incorrect height readings
1. Verify `GANTRY_Z_OFFSET` is calibrated correctly
2. Check that laser and gantry coordinate systems are aligned
3. Ensure the laser scanner is properly calibrated

### No points appearing
- Check intensity threshold (currently 50) - may need adjustment
- Verify laser scanner is detecting objects
- Check that gantry is moving (Y position changing)

## Customization

### Changing the Colormap

```python
# In realtime_2d_scanner.py
self.colorbar = pg.ColorBarItem(
    colorMap=pg.colormap.get('jet'),  # Try: 'jet', 'hot', 'cool', 'plasma'
    ...
)
```

### Adjusting Point Size

```python
# In realtime_2d_scanner.py
self.scatter = pg.ScatterPlotItem(
    size=5,  # Increase for larger points
    ...
)
```

### Changing Memory Limits

```python
MAX_SCAN_POINTS = 20000  # Store more points (uses more RAM)
```

## Technical Details

### Thread Safety
Both versions use threading locks to ensure safe access to shared data between threads:
- Laser acquisition thread
- Gantry monitoring thread
- Main visualization thread

### Performance
- **Full version**: ~50 Hz visualization, ~1000 Hz laser acquisition
- **Simple version**: ~10 Hz visualization, ~1000 Hz laser acquisition

### Memory Usage
Approximate memory per point: 24 bytes (3 floats × 8 bytes)
- 10,000 points ≈ 240 KB
- 100,000 points ≈ 2.4 MB

## License

This software is provided as-is for use with Micro-Epsilon scanCONTROL laser scanners and CRI-compatible gantry systems.

## Support

For issues or questions:
1. Check the troubleshooting section
2. Verify hardware connections
3. Review the configuration settings
4. Check console output for error messages
