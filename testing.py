import time
import sys
from typing import Optional
import math
import urx

# Import our modules
import robot_functions as rf
import urscript_paths as paths

# Configuration
ROBOT_IP = "192.168.0.101"
DEFAULT_WAIT_TIME = 30  # seconds to wait for program completion

HOMETest = [math.radians(a) for a in [201.64, -54.86, 109.85, 214.57, 269.77, 111.69]]

robot = urx.Robot(ROBOT_IP)
rf.set_tcp_offset(robot, -257.81, 0, 60.3, 0, 0, 0)
rf.move_to_joint_position(robot, HOMETest, acc=0.5, vel=0.5)
rf.translate_tcp(robot, dx_mm=0, dy_mm=0, dz_mm=-400, acc=0.5, vel=0.5)

# Conical sweep keeping blue axis (Z) always downward
rf.conical_motion(
    robot,
    tilt_deg=10,        # half-angle of cone
    revolutions=1.0,    # one full spin
    steps=120,
    acc=1,
    vel=1,
    blend_mm=0.001,
    sing_tol_deg=2,
    debug=True,
)


time.sleep(10)

robot.close()




