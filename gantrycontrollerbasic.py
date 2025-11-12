import sys

sys.path.append(r"C:\path\to\python_bindings")
from libs.cri_controller import CRIController

robot = CRIController()
if not robot.connect("192.168.0.10", 3920):
    raise SystemExit("Failed to connect")

try:
    # Wait until kinematics ready (listens for STATUS in background)
    ready = robot.wait_for_kinematics_ready(timeout=30.0)
    if not ready:
        raise RuntimeError("Kinematics not ready")

    # Enable drives
    if not robot.enable():
        raise RuntimeError("Enable failed")

    # Absolute joint move at 20% speed, wait for finish
    ok = robot.move_joints(
        A1=0, A2=0, A3=0,
        A4=0, A5=0, A6=0,
        E1=0, E2=0, E3=0,
        velocity=20.0,
        wait_move_finished=True,
        move_finished_timeout=120.0,
    )
    print("Move result:", ok)
finally:
    robot.disable()
    robot.close()