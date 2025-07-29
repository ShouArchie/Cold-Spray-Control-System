import time
import sys
from typing import Optional
import math
import urx

# Import our modules
import robot_functions as rf
import urscript_paths as paths

# Configuration
ROBOT_IP = "192.168.10.223"
DEFAULT_WAIT_TIME = 30  # seconds to wait for program completion

HOMETest = [math.radians(a) for a in [201.64, -54.86, 109.85, 214.57, 269.77, 111.69]]

robot = urx.Robot(ROBOT_IP)
rf.set_tcp_offset(robot, -257.81, 0, 60.3, 0, 0, 0)
# --- Move down, then MoveP back to home TCP ----------------------
rf.move_to_joint_position(robot, HOMETest, acc=0.5, vel=0.5)
# remember TCP pose of HOMETest (target of MoveP)
home_tcp = robot.getl()

# push tool 400 mm straight down so we have some distance to travel
rf.translate_tcp(robot, dz_mm=-400, acc=0.5, vel=0.5)

# -----------------------------------------------------------------
# Build URScript: multi-waypoint MoveP back to HOME
#   p1 = 200 mm below home
#   p2 = 100 mm below home + 50 mm in Y
#   p3 = exact home pose
# -----------------------------------------------------------------

def shifted_pose(base, dx=0.0, dy=0.0, dz=0.0):
    """Utility: returns base pose translated in base frame (metres)."""
    return [base[0] + dx, base[1] + dy, base[2] + dz] + base[3:]

p1 = shifted_pose(home_tcp, dz=-0.20)               # 200 mm below
p2 = shifted_pose(home_tcp, dy=0.05, dz=-0.10)      # 100 mm below +50 mm Y
p3 = home_tcp                                       # final home pose

def pose_to_str(p):
    return ", ".join(f"{v:.6f}" for v in p)

lines = [
    "def movep_path():",
    f"  movep(p[{pose_to_str(p1)}], a=0.5, v=0.25, r=0.05)",
    f"  movep(p[{pose_to_str(p2)}], a=0.5, v=0.25, r=0.05)",
    f"  movep(p[{pose_to_str(p3)}], a=0.5, v=0.25, r=0.05)",
    "end",
    "movep_path()"
]

script = "\n".join(lines)

rf.send_urscript(robot, script)

# wait a bit to observe
time.sleep(6)

# Conical sweep using movej version (commented out for now)
# rf.conical_motion_script(
#     robot,
#     tilt_deg=10,
#     revolutions=1.0,
#     steps=60,
#     acc=1,
#     vel=1,
#     blend_mm=20,
#     sing_tol_deg=2,
# )


time.sleep(10)

robot.close()




