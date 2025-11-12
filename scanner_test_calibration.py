"""
Test and calibration utility for the 2D laser scanner system.
Use this to verify connections and calibrate offsets.
"""

import sys
import time
import ctypes as ct

sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector")
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector\libs\python_bindings")

try:
    import pyllt as llt
except ImportError:
    from pyllt import pyllt as llt

from libs.cri_lib.cri_controller import CRIController


def test_laser_connection():
    """Test laser scanner connection."""
    print("\n" + "=" * 60)
    print("LASER SCANNER CONNECTION TEST")
    print("=" * 60)
    
    hLLT = None
    try:
        print("\n1. Creating device handle...")
        hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
        if not hLLT:
            print("   ❌ FAILED: Could not create device handle")
            return False
        print("   ✅ Device handle created")
        
        print("\n2. Searching for scanner...")
        interfaces = (ct.c_uint * 6)()
        res = llt.get_device_interfaces_fast(hLLT, interfaces, len(interfaces))
        if res < 1:
            print("   ❌ FAILED: No interfaces found")
            return False
        
        detected = interfaces[0]
        if detected == 0:
            print("   ❌ FAILED: No scanner detected")
            return False
        
        ip_str = f"{detected & 0xFF}.{(detected >> 8) & 0xFF}.{(detected >> 16) & 0xFF}.{(detected >> 24) & 0xFF}"
        print(f"   ✅ Scanner found at: {ip_str}")
        
        print("\n3. Setting device interface...")
        res = llt.set_device_interface(hLLT, detected, 0)
        if res < 1:
            print(f"   ❌ FAILED: set_device_interface returned {res}")
            return False
        print("   ✅ Interface configured")
        
        print("\n4. Connecting...")
        res = llt.connect(hLLT)
        if res < 1:
            print(f"   ❌ FAILED: Connection failed with code {res}")
            if res == -301:
                print("      Scanner may be in use by another application")
            elif res == -303:
                print("      Handle already in use (restart Python)")
            return False
        print("   ✅ Connected successfully")
        
        print("\n5. Getting scanner info...")
        scanner_type = ct.c_int(0)
        res = llt.get_llt_type(hLLT, ct.byref(scanner_type))
        if res < 1:
            print("   ❌ FAILED: Could not get scanner type")
            return False
        print(f"   ✅ Scanner type: {scanner_type.value}")
        
        available_resolutions = (ct.c_uint * 4)()
        res = llt.get_resolutions(hLLT, available_resolutions, len(available_resolutions))
        if res < 1:
            print("   ❌ FAILED: Could not get resolutions")
            return False
        
        print(f"   ✅ Available resolutions: {[int(r) for r in available_resolutions if int(r) > 0]}")
        
        print("\n6. Testing data acquisition...")
        resolution = int(available_resolutions[0])
        llt.set_resolution(hLLT, resolution)
        llt.set_profile_config(hLLT, llt.TProfileConfig.PROFILE)
        
        # Start transfer
        res = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        if res < 1:
            print(f"   ❌ FAILED: Could not start transfer ({res})")
            return False
        
        # Try to get a profile
        profile_buffer = (ct.c_ubyte * (resolution * 64))()
        lost_profiles = ct.c_int(0)
        
        success = False
        for attempt in range(10):
            res = llt.get_actual_profile(
                hLLT, profile_buffer, len(profile_buffer),
                llt.TProfileConfig.PROFILE, ct.byref(lost_profiles)
            )
            if res >= 1:
                success = True
                break
            time.sleep(0.1)
        
        llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
        
        if success:
            print(f"   ✅ Successfully acquired profile (resolution: {resolution})")
        else:
            print("   ❌ FAILED: Could not acquire profile")
            return False
        
        print("\n✅ LASER SCANNER TEST PASSED")
        return True
        
    except Exception as e:
        print(f"\n❌ EXCEPTION: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        if hLLT:
            try:
                llt.disconnect(hLLT)
            except Exception:
                pass


def test_gantry_connection():
    """Test gantry connection."""
    print("\n" + "=" * 60)
    print("GANTRY CONNECTION TEST")
    print("=" * 60)
    
    robot = CRIController()
    
    try:
        print("\n1. Connecting to gantry...")
        print(f"   IP: 192.168.3.11")
        print(f"   Port: 3920")
        
        if not robot.connect("192.168.3.11", 3920):
            print("   ❌ FAILED: Could not connect")
            return False
        print("   ✅ Connected")
        
        print("\n2. Waiting for status update...")
        try:
            robot.wait_for_status_update(timeout=3.0)
            print("   ✅ Status received")
        except Exception as e:
            print(f"   ⚠️  WARNING: Status timeout ({e})")
        
        print("\n3. Reading position...")
        try:
            pos = robot.robot_state.position_robot
            print(f"   ✅ Raw position:")
            print(f"      X = {pos.X:.3f}")
            print(f"      Y = {pos.Y:.3f}")
            print(f"      Z = {pos.Z:.3f}")
            
            # With offsets
            print(f"\n   With current offsets (-0.9, -1.0, +1.3):")
            print(f"      X = {pos.X - 0.9:.3f}")
            print(f"      Y = {pos.Y - 1.0:.3f}")
            print(f"      Z = {pos.Z + 1.3:.3f}")
            
        except Exception as e:
            print(f"   ❌ FAILED: Could not read position ({e})")
            return False
        
        print("\n✅ GANTRY TEST PASSED")
        return True
        
    except Exception as e:
        print(f"\n❌ EXCEPTION: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        try:
            robot.close()
        except Exception:
            pass


def calibrate_offsets():
    """Interactive offset calibration."""
    print("\n" + "=" * 60)
    print("GANTRY OFFSET CALIBRATION")
    print("=" * 60)
    
    robot = CRIController()
    
    try:
        if not robot.connect("192.168.3.11", 3920):
            print("❌ Could not connect to gantry")
            return
        
        print("\nInstructions:")
        print("1. Move the gantry to a known reference position")
        print("2. Enter the reference position coordinates")
        print("3. The tool will calculate the required offsets")
        
        input("\nPress Enter when gantry is at reference position...")
        
        # Get current position
        try:
            robot.wait_for_status_update(timeout=2.0)
        except Exception:
            pass
        
        pos = robot.robot_state.position_robot
        
        print(f"\nCurrent robot position:")
        print(f"  X = {pos.X:.3f}")
        print(f"  Y = {pos.Y:.3f}")
        print(f"  Z = {pos.Z:.3f}")
        
        print("\nEnter the reference (world) coordinates:")
        try:
            ref_x = float(input("  Reference X: "))
            ref_y = float(input("  Reference Y: "))
            ref_z = float(input("  Reference Z: "))
        except ValueError:
            print("❌ Invalid input")
            return
        
        # Calculate offsets
        offset_x = ref_x - pos.X
        offset_y = ref_y - pos.Y
        offset_z = ref_z - pos.Z
        
        print("\n" + "=" * 60)
        print("CALCULATED OFFSETS:")
        print("=" * 60)
        print(f"\nGANTRY_X_OFFSET = {offset_x:.3f}")
        print(f"GANTRY_Y_OFFSET = {offset_y:.3f}")
        print(f"GANTRY_Z_OFFSET = {offset_z:.3f}")
        
        print("\nUpdate these values in your scanner script!")
        print("=" * 60)
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        try:
            robot.close()
        except Exception:
            pass


def monitor_positions():
    """Monitor laser and gantry positions in real-time."""
    print("\n" + "=" * 60)
    print("REAL-TIME POSITION MONITOR")
    print("=" * 60)
    print("\nPress Ctrl+C to exit\n")
    
    # Connect laser
    hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
    interfaces = (ct.c_uint * 6)()
    llt.get_device_interfaces_fast(hLLT, interfaces, len(interfaces))
    llt.set_device_interface(hLLT, interfaces[0], 0)
    llt.connect(hLLT)
    
    scanner_type = ct.c_int(0)
    llt.get_llt_type(hLLT, ct.byref(scanner_type))
    
    av = (ct.c_uint * 4)()
    llt.get_resolutions(hLLT, av, len(av))
    resolution = int(av[0])
    llt.set_resolution(hLLT, resolution)
    llt.set_profile_config(hLLT, llt.TProfileConfig.PROFILE)
    llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
    
    profile_buffer = (ct.c_ubyte * (resolution * 64))()
    lost = ct.c_int(0)
    x_buf = (ct.c_double * resolution)()
    z_buf = (ct.c_double * resolution)()
    i_buf = (ct.c_ushort * resolution)()
    snull_us = ct.POINTER(ct.c_ushort)()
    snull_ui = ct.POINTER(ct.c_uint)()
    
    # Connect gantry
    robot = CRIController()
    robot.connect("192.168.3.11", 3920)
    
    try:
        import numpy as np
        
        while True:
            # Get laser data
            r = llt.get_actual_profile(hLLT, profile_buffer, len(profile_buffer),
                                       llt.TProfileConfig.PROFILE, ct.byref(lost))
            
            if r >= 1:
                llt.convert_profile_2_values(
                    hLLT, profile_buffer, resolution, llt.TProfileConfig.PROFILE,
                    scanner_type.value, 0, 1, snull_us, i_buf, snull_us,
                    x_buf, z_buf, snull_ui, snull_ui
                )
                
                z_np = np.frombuffer(z_buf, dtype=np.float64, count=resolution)
                i_np = np.frombuffer(i_buf, dtype=np.uint16, count=resolution)
                
                valid = (i_np > 50) & (z_np > 0) & np.isfinite(z_np)
                
                if valid.any():
                    z_mean = z_np[valid].mean()
                    z_min = z_np[valid].min()
                    z_max = z_np[valid].max()
                    
                    # Get gantry position
                    pos = robot.robot_state.position_robot
                    gx = pos.X - 0.9
                    gy = pos.Y - 1.0
                    gz = pos.Z + 1.3
                    
                    # Compensated height
                    comp_mean = z_mean - gz
                    
                    print(f"\rGantry: X={gx:7.2f} Y={gy:7.2f} Z={gz:7.2f} | "
                          f"Laser Z: {z_mean:7.2f} (min={z_min:6.2f}, max={z_max:6.2f}) | "
                          f"Height: {comp_mean:7.2f} mm", end='')
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
        llt.disconnect(hLLT)
        robot.close()


def main():
    """Main menu."""
    while True:
        print("\n" + "=" * 60)
        print("2D LASER SCANNER - TEST & CALIBRATION UTILITY")
        print("=" * 60)
        print("\n1. Test laser scanner connection")
        print("2. Test gantry connection")
        print("3. Calibrate gantry offsets")
        print("4. Monitor positions (real-time)")
        print("5. Run all tests")
        print("0. Exit")
        
        choice = input("\nEnter choice: ").strip()
        
        if choice == '1':
            test_laser_connection()
        elif choice == '2':
            test_gantry_connection()
        elif choice == '3':
            calibrate_offsets()
        elif choice == '4':
            monitor_positions()
        elif choice == '5':
            laser_ok = test_laser_connection()
            gantry_ok = test_gantry_connection()
            print("\n" + "=" * 60)
            print("OVERALL RESULTS:")
            print("=" * 60)
            print(f"Laser Scanner: {'✅ PASS' if laser_ok else '❌ FAIL'}")
            print(f"Gantry:        {'✅ PASS' if gantry_ok else '❌ FAIL'}")
            if laser_ok and gantry_ok:
                print("\n✅ System ready for scanning!")
            else:
                print("\n❌ Please fix issues before scanning")
        elif choice == '0':
            print("\nExiting...")
            break
        else:
            print("Invalid choice")


if __name__ == "__main__":
    main()
