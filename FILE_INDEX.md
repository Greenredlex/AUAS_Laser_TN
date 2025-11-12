# 2D Laser Scanner System - File Index

## 📁 Project Files Overview

### 🎯 Main Applications (Choose One)

| File | Description | When to Use |
|------|-------------|-------------|
| **`realtime_2d_scanner.py`** | ⭐ **RECOMMENDED** - Full-featured PyQtGraph version | Production, high frame rate scanning |
| **`realtime_2d_scanner_simple.py`** | Simplified Matplotlib version | Learning, testing, simpler setup |

---

### 🔧 Utilities

| File | Description | Purpose |
|------|-------------|---------|
| **`scanner_test_calibration.py`** | Interactive test & calibration tool | Hardware verification, offset calibration |
| **`visual_concept_demo.py`** | Visual concept demonstration | Understanding how the system works |

---

### 📚 Documentation

| File | Description | Read When |
|------|-------------|-----------|
| **`QUICKSTART.md`** | ⚡ Quick start guide (3 steps) | **START HERE** - First time setup |
| **`PROJECT_SUMMARY.md`** | Complete project overview | Understanding the whole system |
| **`README_2D_SCANNER.md`** | Technical documentation | Detailed information, customization |
| **`FILE_INDEX.md`** | This file | Finding what you need |

---

### 🔄 Existing Files (Your Original Work)

| File | Description |
|------|-------------|
| `contiuouslasertest.py` | Your laser scanner test code |
| `gantryrealtimecontroller.py` | Your gantry joystick controller |
| `gantrycontrollerbasic.py` | Basic gantry control |
| `run_scancontrol_waterfall_intensity.py` | Waterfall visualization |
| `Scanvierwer.py` | Scan viewer |

---

## 🚀 Quick Start Path

```
1. Read: QUICKSTART.md
   ↓
2. Run: scanner_test_calibration.py (verify hardware)
   ↓
3. Run: scanner_test_calibration.py (calibrate offsets)
   ↓
4. Update offsets in: realtime_2d_scanner.py
   ↓
5. Run: realtime_2d_scanner.py
   ↓
6. SCAN! 🎉
```

---

## 📖 Learning Path

```
1. Read: PROJECT_SUMMARY.md (understand concept)
   ↓
2. Run: visual_concept_demo.py (see visualizations)
   ↓
3. Read: README_2D_SCANNER.md (technical details)
   ↓
4. Study: realtime_2d_scanner_simple.py (simpler code)
   ↓
5. Study: realtime_2d_scanner.py (full implementation)
```

---

## 🔍 Finding Information

### "How do I...?"

| Question | Look Here |
|----------|-----------|
| Get started quickly? | `QUICKSTART.md` |
| Test my hardware? | `scanner_test_calibration.py` → Option 5 |
| Calibrate offsets? | `scanner_test_calibration.py` → Option 3 |
| Understand the concept? | `PROJECT_SUMMARY.md` or `visual_concept_demo.py` |
| Fix a problem? | `QUICKSTART.md` → Common Issues section |
| Customize the code? | `README_2D_SCANNER.md` → Customization section |
| See live data? | `scanner_test_calibration.py` → Option 4 |
| Understand the architecture? | `PROJECT_SUMMARY.md` → Architecture section |
| Change colormap? | `README_2D_SCANNER.md` → Changing the Colormap |
| Improve performance? | `README_2D_SCANNER.md` → Performance section |

---

## 🎯 By Use Case

### First Time Setup
1. `QUICKSTART.md` - Read this
2. `scanner_test_calibration.py` - Run tests
3. `scanner_test_calibration.py` - Calibrate
4. `realtime_2d_scanner.py` - Update and run

### Daily Scanning
1. `realtime_2d_scanner.py` - Run scanner
2. Move gantry to scan
3. Done!

### Troubleshooting
1. `scanner_test_calibration.py` - Diagnose issue
2. `QUICKSTART.md` - Check common issues
3. `README_2D_SCANNER.md` - Detailed troubleshooting

### Learning/Understanding
1. `visual_concept_demo.py` - See concepts
2. `PROJECT_SUMMARY.md` - Read overview
3. `realtime_2d_scanner_simple.py` - Study code

### Customization
1. `README_2D_SCANNER.md` - Customization guide
2. `realtime_2d_scanner.py` - Modify code
3. Test changes

---

## 📊 File Complexity

### Beginner-Friendly
- ⭐⭐⭐ `QUICKSTART.md` - Easy reading
- ⭐⭐⭐ `PROJECT_SUMMARY.md` - Easy reading
- ⭐⭐☆ `visual_concept_demo.py` - Visual, easy to understand
- ⭐⭐☆ `scanner_test_calibration.py` - Interactive, guided

### Intermediate
- ⭐⭐⭐ `realtime_2d_scanner_simple.py` - Clean code, well commented
- ⭐⭐⭐ `README_2D_SCANNER.md` - Technical but clear

### Advanced
- ⭐⭐⭐⭐ `realtime_2d_scanner.py` - Multi-threaded, complex
- ⭐⭐⭐⭐ Architecture understanding - Threading, synchronization

---

## 🎨 File Dependencies

```
realtime_2d_scanner.py
├── numpy
├── pyqtgraph
├── PyQt5 (or PySide6)
├── pyllt (laser scanner)
└── libs.cri_lib (gantry)

realtime_2d_scanner_simple.py
├── numpy
├── matplotlib
├── pyllt (laser scanner)
└── libs.cri_lib (gantry)

scanner_test_calibration.py
├── numpy
├── pyllt (laser scanner)
└── libs.cri_lib (gantry)

visual_concept_demo.py
├── numpy
└── matplotlib
```

---

## 📏 File Sizes (Approximate)

| File | Lines | Size | Complexity |
|------|-------|------|------------|
| `realtime_2d_scanner.py` | ~550 | 20 KB | High |
| `realtime_2d_scanner_simple.py` | ~300 | 12 KB | Medium |
| `scanner_test_calibration.py` | ~350 | 14 KB | Low |
| `visual_concept_demo.py` | ~250 | 10 KB | Low |
| `QUICKSTART.md` | ~350 | 10 KB | - |
| `PROJECT_SUMMARY.md` | ~400 | 12 KB | - |
| `README_2D_SCANNER.md` | ~600 | 18 KB | - |

---

## 🔑 Key Code Sections

### To Find Specific Features

#### Laser Scanner Connection
- `realtime_2d_scanner.py` → `LaserScannerThread.connect()`
- Lines ~80-130

#### Gantry Connection
- `realtime_2d_scanner.py` → `GantryMonitorThread.connect()`
- Lines ~240-260

#### Height Compensation
- `realtime_2d_scanner.py` → `update_visualization()`
- Line ~398: `compensated_z = valid_z - gantry_z`

#### Visualization Setup
- `realtime_2d_scanner.py` → `ScanVisualizerWindow.setup_ui()`
- Lines ~305-360

#### Configuration Constants
- **ALL SCRIPTS**: Top section (after imports)
- Look for `GANTRY_IP`, `GANTRY_X_OFFSET`, etc.

---

## 🆘 Problem-Solution Map

| Problem | File to Check | Section |
|---------|---------------|---------|
| Can't connect to laser | `scanner_test_calibration.py` | Option 1 |
| Can't connect to gantry | `scanner_test_calibration.py` | Option 2 |
| Wrong heights | `scanner_test_calibration.py` | Option 3 (calibrate) |
| No data showing | `QUICKSTART.md` | Troubleshooting |
| Slow performance | `README_2D_SCANNER.md` | Performance Tips |
| Code not working | `PROJECT_SUMMARY.md` | Testing Checklist |
| Don't understand concept | `visual_concept_demo.py` | Run it! |
| Need to customize | `README_2D_SCANNER.md` | Customization |

---

## 📞 Support Flow

```
Problem?
  ↓
Run: scanner_test_calibration.py → Option 5
  ↓
Still broken? Check: QUICKSTART.md → Common Issues
  ↓
Still broken? Read: README_2D_SCANNER.md → Troubleshooting
  ↓
Still broken? Check console output for error messages
  ↓
Still broken? Review: PROJECT_SUMMARY.md → Architecture
  ↓
Still broken? Check hardware connections & power
```

---

## 🎓 Educational Value

### Learn About:

| Topic | Where to Learn |
|-------|----------------|
| Threading in Python | `realtime_2d_scanner.py` - Thread classes |
| Real-time visualization | Both scanner scripts |
| Hardware interfacing | Laser/Gantry thread classes |
| PyQtGraph usage | `realtime_2d_scanner.py` |
| Matplotlib animation | `realtime_2d_scanner_simple.py` |
| Thread synchronization | Lock usage throughout |
| Error handling | Try/except blocks throughout |
| GUI design | `ScanVisualizerWindow` class |

---

## ✅ Recommended Reading Order

### For Quick Setup (30 minutes)
1. `QUICKSTART.md` (5 min)
2. Run `scanner_test_calibration.py` (10 min)
3. Calibrate offsets (10 min)
4. Run `realtime_2d_scanner.py` (5 min)

### For Understanding (2 hours)
1. `PROJECT_SUMMARY.md` (20 min)
2. `visual_concept_demo.py` (15 min)
3. `QUICKSTART.md` (10 min)
4. `README_2D_SCANNER.md` (30 min)
5. Study `realtime_2d_scanner_simple.py` (30 min)
6. Experiment with modifications (15 min)

### For Mastery (1 day)
1. All documentation (2 hours)
2. Study all code files (3 hours)
3. Modify and test (2 hours)
4. Create custom features (2 hours)

---

## 🌟 Most Important Files

If you only read 3 files:
1. **`QUICKSTART.md`** - Get running quickly
2. **`PROJECT_SUMMARY.md`** - Understand the system
3. **`realtime_2d_scanner.py`** - The main application

---

## 📝 Quick Reference Card

### Essential Commands
```powershell
# Setup
pip install numpy matplotlib pyqtgraph PyQt5

# Test
python scanner_test_calibration.py

# Run
python realtime_2d_scanner.py
```

### Essential Files
- **Run**: `realtime_2d_scanner.py`
- **Test**: `scanner_test_calibration.py`
- **Learn**: `QUICKSTART.md`

### Essential Settings
```python
GANTRY_X_OFFSET = -0.9  # Calibrate this!
GANTRY_Y_OFFSET = -1.0  # Calibrate this!
GANTRY_Z_OFFSET = 1.3   # Calibrate this!
```

---

**Ready to start? Open `QUICKSTART.md`! 🚀**
