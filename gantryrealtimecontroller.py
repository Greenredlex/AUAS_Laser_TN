import sys
import time
import threading

# Adjust import path if needed
sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector")

import keyboard  # pip install keyboard
from libs.cri_lib.cri_controller import CRIController
from libs.cri_lib.robot_state import KinematicsState

IP = "192.168.3.11"
PORT = 3920

# Jog parameters
JOG_SPEED_STEP = 50.0     # percent of max per key press (held keys keep dwadseqeqit applied)
JOG_UPDATE_HZ = 50        # 50 Hz jog updates
JOG_DT = 1.0 / JOG_UPDATE_HZ

def main():
    robot = CRIController()
    active = enabled = False
    jog_active = False

    # Shared speeds
    speeds = {"A1": 0.0, "A2": 0.0, "A3": 0.0}
    stop_flag = {"stop": False}

    def key_loop():
        # No blocking input; we poll key states
        # Hold keys to keep motion, release to stop that axis
        while not stop_flag["stop"]:
            # Exit
            if keyboard.is_pressed("esc"):
                stop_flag["stop"] = True
                break

            # Space zeroes all
            if keyboard.is_pressed("space"):
                speeds["A1"] = speeds["A2"] = speeds["A3"] = 0.0

            # A1: w/s
            if keyboard.is_pressed("w"):
                speeds["A1"] = +JOG_SPEED_STEP
            elif keyboard.is_pressed("s"):
                speeds["A1"] = -JOG_SPEED_STEP
            else:
                # If neither pressed, zero that axis
                speeds["A1"] = 0.0

            # A2: a/d
            if keyboard.is_pressed("d"):
                speeds["A2"] = +JOG_SPEED_STEP
            elif keyboard.is_pressed("a"):
                speeds["A2"] = -JOG_SPEED_STEP
            else:
                speeds["A2"] = 0.0

            # A3: q/e
            if keyboard.is_pressed("q"):
                speeds["A3"] = +JOG_SPEED_STEP
            elif keyboard.is_pressed("e"):
                speeds["A3"] = -JOG_SPEED_STEP
            else:
                speeds["A3"] = 0.0

            pos = robot.robot_state.position_robot
            x = pos.X - 0.9
            y = pos.Y - 1.0
            z = pos.Z + 1.3
            print("Cart pos:", x,y,z)

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

        # Start keyboard thread
        t = threading.Thread(target=key_loop, daemon=True)
        t.start()

        print("WASDQE to jog A1/A2/A3, SPACE to zero, ESC to exit.")
        while not stop_flag["stop"]:
            robot.set_jog_values(
                A1=speeds["A1"],
                A2=speeds["A2"],
                A3=speeds["A3"],
                A4=0.0, A5=0.0, A6=0.0,
                E1=0.0, E2=0.0, E3=0.0,
            )
            time.sleep(JOG_DT)

    finally:
        # Clean shutdown
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
        print("Exited cleanly.")

if __name__ == "__main__":
    main()