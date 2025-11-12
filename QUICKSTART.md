# Quick Start Guide - 2D Laser Scanner

## 🚀 Getting Started in 3 Steps

### Step 1: Install Dependencies

```powershell
pip install numpy matplotlib pyqtgraph PyQt5 keyboard
```

### Step 2: Test Your Hardware

```powershell
python scanner_test_calibration.py
```

Select option **5** (Run all tests) to verify both the laser scanner and gantry are working.

### Step 3: Run the Scanner

```powershell
# For best performance (PyQtGraph):
python realtime_2d_scanner.py

# OR for simple version (Matplotlib):
python realtime_2d_scanner_simple.py
```

---

## 📋 Available Scripts

| Script | Description | Best For |
|--------|-------------|----------|
| `realtime_2d_scanner.py` | Full-featured version with PyQtGraph | Production use, high frame rates |
| `realtime_2d_scanner_simple.py` | Simplified Matplotlib version | Learning, testing, debugging |
| `scanner_test_calibration.py` | Test & calibration utility | Setup, troubleshooting |

---

## 🔧 Calibration (Important!)

### Why Calibrate?

The scanner needs to know how to convert robot coordinates to world coordinates. The offsets compensate for the difference between where the robot thinks it is and where your reference frame is.

### How to Calibrate

1. Run the calibration utility:
   ```powershell
   python scanner_test_calibration.py
   ```

2. Select option **3** (Calibrate gantry offsets)

3. Move the gantry to a known reference position (e.g., origin)

4. Enter the reference coordinates when prompted

5. Copy the calculated offsets into your scanner script:
   ```python
   GANTRY_X_OFFSET = -0.9  # Replace with your value
   GANTRY_Y_OFFSET = -1.0  # Replace with your value
   GANTRY_Z_OFFSET = 1.3   # Replace with your value
   ```

---

## 🎮 Usage

### Real-time Scanning

1. **Start the scanner application**
   ```powershell
   python realtime_2d_scanner.py
   ```

2. **Move the gantry** (Y-axis) to scan across your object
   - The laser scans along X-axis automatically
   - Gantry movement creates the Y-axis data
   - Height is shown in color

3. **Controls**:
   - **Pause** button: Pause data acquisition
   - **Clear Data** button: Reset the visualization
   - **Close window**: Exit the application

### Viewing Results

- **X-axis**: Position along the laser line (mm)
- **Y-axis**: Gantry Y position (mm)  
- **Color**: Object height (mm)
  - Blue = Low
  - Green/Yellow = Medium
  - Red = High

---

## 🎯 How It Works

### Height Compensation Explained

```
┌─────────────────────────────────────────┐
│         Laser Scanner (moving)          │
│              ↓ Z reading                │
├─────────────────────────────────────────┤
│              Object                     │
│         (what we want to measure)       │
├─────────────────────────────────────────┤
│              Table                      │
└─────────────────────────────────────────┘

Object Height = Laser Z Reading - Gantry Z Position
```

**Without compensation**: If you move the laser up, objects appear taller
**With compensation**: Object height stays constant regardless of laser position

---

## 📊 Coordinate System

```
         Z (up)
         ↑
         |
         |
         └────→ X (laser scan line)
        /
       /
      Y (gantry movement)
```

- **X**: Perpendicular to gantry movement (laser scan line)
- **Y**: Direction of gantry travel
- **Z**: Height (vertical)

---

## 🐛 Common Issues

### "No laser scanner found"
**Solution**: 
- Check scanner is powered on
- Verify network connection
- Close Micro-Epsilon configuration software

### "Failed to connect to gantry"
**Solution**:
- Verify IP: `192.168.3.11`
- Check port: `3920`
- Ensure gantry is powered on

### Objects have wrong height
**Solution**:
- Run calibration: `python scanner_test_calibration.py`
- Verify offsets are correct
- Check laser scanner calibration

### Low frame rate / laggy
**Solution**:
- Use `realtime_2d_scanner.py` (PyQtGraph version)
- Reduce `MAX_SCAN_POINTS` in script
- Increase `UPDATE_RATE_MS` in script

### No data appearing
**Solution**:
- Check laser is detecting objects (run monitor in test utility)
- Verify gantry is moving (Y position changing)
- Lower intensity threshold in code (currently 50)

---

## 🔍 Testing & Monitoring

### Real-time Position Monitor

See live laser and gantry data:

```powershell
python scanner_test_calibration.py
# Select option 4
```

This shows:
- Gantry X, Y, Z position
- Laser Z reading (min, max, mean)
- Compensated height

Press `Ctrl+C` to exit.

---

## ⚙️ Configuration

### Key Settings (in scanner scripts)

```python
# Connection
GANTRY_IP = "192.168.3.11"
GANTRY_PORT = 3920

# Calibration (CUSTOMIZE THIS!)
GANTRY_X_OFFSET = -0.9
GANTRY_Y_OFFSET = -1.0
GANTRY_Z_OFFSET = 1.3

# Performance
MAX_SCAN_POINTS = 10000  # More = more memory, slower
UPDATE_RATE_MS = 50      # Lower = faster updates, more CPU
```

### Intensity Threshold

If you're not seeing enough points, lower this:

```python
# In the scripts, find this line:
valid_mask = (intensity > 50) & ...  # Change 50 to lower value
```

---

## 📝 Example Workflow

### Scanning a Flat Object

1. Start scanner: `python realtime_2d_scanner.py`
2. Click "Clear Data" to reset
3. Move gantry from Y=0 to Y=100mm
4. Watch the 2D map fill in
5. Colors should be uniform if object is flat

### Scanning a Height Step

1. Place object with step
2. Start scanner
3. Move gantry across the step
4. You should see color change at the step

### Scanning Multiple Objects

1. Start scanner  
2. Click "Clear Data"
3. Scan first object by moving gantry
4. Move to second object
5. Continue scanning (data accumulates)
6. Click "Clear Data" when starting new scan

---

## 💡 Tips

### For Best Results

✅ Move gantry smoothly and consistently  
✅ Ensure good lighting for laser scanner  
✅ Calibrate offsets before first use  
✅ Use "Pause" when repositioning without scanning  
✅ Clear data between different scans  

### Performance Tips

✅ Use PyQtGraph version for real-time scanning  
✅ Close other applications using network  
✅ Reduce point count for faster rendering  

### Accuracy Tips

✅ Calibrate at multiple positions and average  
✅ Check laser scanner calibration  
✅ Avoid scanning reflective surfaces  
✅ Keep gantry movement speed constant  

---

## 📚 Next Steps

1. ✅ Run hardware tests
2. ✅ Calibrate offsets  
3. ✅ Test with simple object
4. ✅ Adjust visualization settings
5. ✅ Scan real objects!

For detailed information, see `README_2D_SCANNER.md`.

---

## 🆘 Need Help?

1. Run diagnostic: `python scanner_test_calibration.py` → Option 5
2. Check console output for error messages
3. Review `README_2D_SCANNER.md` for details
4. Verify hardware connections

---

**Happy Scanning! 📡✨**
