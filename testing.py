import time
import sys
from typing import Optional
import math
import urx

# Import our modules
import robot_functions as rf
import urscript_paths as paths

# Configuration
ROBOT_IP = "192.168.10.205"
DEFAULT_WAIT_TIME = 30  # seconds to wait for program completion

HOMETest = [math.radians(a) for a in [201.64, -54.86, 109.85, 214.57, 269.77, 111.69]]

robot = urx.Robot(ROBOT_IP)
rf.set_tcp_offset(robot, -257.81, 0, 60.3, 0, 0, 0)
# --- Move down, then MoveP back to home TCP ----------------------
rf.move_to_joint_position(robot, HOMETest, acc=0.5, vel=0.5)
# remember TCP pose of HOMETest (target of MoveP)
home_tcp = robot.getl()

# push tool 400 mm straight down so we have some distance to travel
# rf.translate_tcp(robot, dz_mm=-400, acc=0.5, vel=0.5)



# Conical sweep using movej version (commented out for now)
rf.conical_motion_servoj_script(
    robot,
    tilt_deg=10,
    revolutions=1.0,
    steps=720,
    cycle_s=0.008,
    lookahead_time=0.05,
    gain=1000,
    sing_tol_deg=2,
)


time.sleep(10)

robot.close()




