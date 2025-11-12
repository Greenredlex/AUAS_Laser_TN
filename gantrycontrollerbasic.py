import sys

sys.path.append(r"C:\Users\BrightSky\Documents\PROJECTS\auas_inspection_engine\scenario_inspector")
import time
from libs.cri_lib.cri_controller import CRIController
from libs.cri_lib.robot_state import KinematicsState

IP = "192.168.3.11"
PORT = 3920

def run_once(xpos,ypos,zpos):
    robot = CRIController()
    active = enabled = False
    try:
        if not robot.connect(IP, PORT):
            print("Connect failed")
            return False

        # Take/retake active control
        active = robot.set_active_control(True)

        # Now enable and move
        enabled = robot.enable()
        if not enabled:
            print("Enable failed")
            return False

        ok = robot.move_joints(
            A1=xpos, A2=ypos, A3=zpos, A4=0, A5=0, A6=0, E1=0, E2=0, E3=0,
            velocity=5.0, wait_move_finished=True, move_finished_timeout=30.0
        )
        print("Move:", ok)
        return ok

    finally:
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

if __name__ == "__main__":
    print("Run 1:", run_once(xpos=100, ypos=20, zpos=30))
    print("Run 2:", run_once(xpos=50, ypos=60, zpos=70))