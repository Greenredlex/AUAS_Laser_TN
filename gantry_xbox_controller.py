import sys
import time
import threading

# Adjust import path if needed
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector")

try:
    import pygame  # pip install pygame
except ImportError:
    print("pygame not installed. Please run: pip install pygame")
    sys.exit(1)

from libs.cri_lib.cri_controller import CRIController
from libs.cri_lib.robot_state import KinematicsState

IP = "192.168.3.11"
PORT = 3920

# Controller parameters
JOG_UPDATE_HZ = 50        # 50 Hz jog updates
JOG_DT = 1.0 / JOG_UPDATE_HZ
MAX_SPEED = 150.0         # Maximum speed percentage
DEADZONE = 0.05           # Deadzone for analog sticks (0.0 to 1.0)

def apply_deadzone(value, deadzone=DEADZONE):
    """Apply deadzone to analog stick input"""
    if abs(value) < deadzone:
        return 0.0
    # Scale the remaining range
    sign = 1 if value > 0 else -1
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return sign * scaled

def main():
    # Initialize pygame and joystick IN MAIN THREAD (important!)
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No Xbox controller detected. Please connect controller and try again.")
        return
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to: {joystick.get_name()}")
    print(f"Axes: {joystick.get_numaxes()}, Buttons: {joystick.get_numbuttons()}")
    
    robot = CRIController()
    active = enabled = False
    jog_active = False

    # Shared speeds and control state
    speeds = {"A1": 0.0, "A2": 0.0, "A3": 0.0}
    stop_flag = {"stop": False}
    robot_ready = {"ready": False}

    def robot_jog_loop():
        """Send jog commands to robot continuously"""
        while not stop_flag["stop"]:
            if robot_ready["ready"]:
                robot.set_jog_values(
                    A1=speeds["A1"],
                    A2=speeds["A2"],
                    A3=speeds["A3"],
                    A4=0.0, A5=0.0, A6=0.0,
                    E1=0.0, E2=0.0, E3=0.0,
                )
            time.sleep(JOG_DT)

    try:
        if not robot.connect(IP, PORT):
            print("Connect failed")
            return

        # Ensure one STATUS and clear Motion-Not-Enabled if needed
        try:
            robot.wait_for_status_update(timeout=2.0)
        except Exception:
            pass

        active = robot.set_active_control(True)

        # If motion not ready, try a reset loop quickly
        ready_deadline = time.time() + 10.0
        while time.time() < ready_deadline:
            robot.wait_for_status_update(timeout=1.0)
            with robot.robot_state_lock:
                rs = robot.robot_state
                kin_ok = (rs.kinematics_state == KinematicsState.NO_ERROR and
                          rs.combined_axes_error == "NoError")
            if kin_ok:
                break
            robot.reset()

        enabled = robot.enable()
        if not enabled:
            print("Enable failed")
            return

        # Start jog mode
        robot.start_jog()
        jog_active = True
        robot_ready["ready"] = True

        # Start robot communication thread
        robot_thread = threading.Thread(target=robot_jog_loop, daemon=True)
        robot_thread.start()

        print("\n" + "="*60)
        print("XBOX CONTROLLER GANTRY CONTROL")
        print("="*60)
        print("Axis 0 (Left Stick X):  X movement (A2)")
        print("Axis 1 (Left Stick Y):  Y movement (A1)")
        print("Axis 3 (Right Stick Y): Z/Height movement (A3)")
        print("Speed: Based on joystick deflection (0-100%)")
        print("B Button:               Emergency stop")
        print("Start:                  Exit program")
        print("="*60 + "\n")

        # MAIN LOOP - Read controller in main thread
        clock = pygame.time.Clock()
        while not stop_flag["stop"]:
            # Process pygame events (MUST be in main thread)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    stop_flag["stop"] = True
            
            # Read raw axis values
            raw_axis_0 = joystick.get_axis(0)
            raw_axis_1 = joystick.get_axis(1)
            raw_axis_3 = joystick.get_axis(3)
            
            # Axis 0: X-axis (A2) - Left stick X
            # Axis 1: Y-axis (A1) - Left stick Y (inverted)
            axis_x = apply_deadzone(raw_axis_1)  # X control
            axis_y = apply_deadzone(-raw_axis_0) # Y control (inverted)
            
            # Axis 3: Height/Z-axis (A3) - Right stick Y (inverted)
            axis_z = apply_deadzone(raw_axis_3) # Z/Height control
            
            # Speed is now based on joystick deflection (0-100%)
            speeds["A1"] = axis_y * MAX_SPEED  # Y-axis
            speeds["A2"] = axis_x * MAX_SPEED  # X-axis
            speeds["A3"] = axis_z * MAX_SPEED  # Z-axis (Height)
            
            # Emergency stop with B button (button 1 on Xbox controller)
            if joystick.get_button(1):  # B button
                speeds["A1"] = speeds["A2"] = speeds["A3"] = 0.0
                print("\nEmergency stop activated!")
            
            # Exit with Start button (button 7)
            if joystick.get_button(7):  # Start button
                stop_flag["stop"] = True
                break
            
            # Display current position and controller values
            try:
                pos = robot.robot_state.position_robot
                x = pos.X - 0.9
                y = pos.Y - 1.0
                z = pos.Z + 1.3
                print(f"Cart: X={x:.3f}, Y={y:.3f}, Z={z:.3f} | Speeds: A1={speeds['A1']:.1f} A2={speeds['A2']:.1f} A3={speeds['A3']:.1f} | RAW: {raw_axis_0:.2f},{raw_axis_1:.2f},{raw_axis_3:.2f}", end='\r')
            except:
                # If robot state fails, still show controller values
                print(f"Speeds: A1={speeds['A1']:.1f} A2={speeds['A2']:.1f} A3={speeds['A3']:.1f} | RAW: ax0={raw_axis_0:.2f} ax1={raw_axis_1:.2f} ax3={raw_axis_3:.2f}", end='\r')
            
            clock.tick(50)  # Run at 50 Hz

    finally:
        # Clean shutdown
        print("\nShutting down...")
        try:
            if jog_active:
                robot.stop_jog()
        except Exception:
            pass
        try:
            robot.stop_move()
        except Exception:
            pass
        if enabled:
            try:
                robot.disable()
            except Exception:
                pass
        if active:
            try:
                robot.set_active_control(False)
            except Exception:
                pass
        try:
            robot.close()
        except Exception:
            pass
        
        pygame.joystick.quit()
        pygame.quit()
        print("Exited cleanly.")

if __name__ == "__main__":
    main()
