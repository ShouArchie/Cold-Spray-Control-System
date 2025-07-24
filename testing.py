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
rf.translate_tcp(robot, dx_mm=0, dy_mm=0, dz_mm=-100, acc=0.5, vel=0.5)
rf.conical_motion_fixed_tcp(
                robot,
                max_angle=10,
                num_points=24,
                tcp_offset_z=-257.81,  # Your tool length
                acc=0.1,
                vel=0.1,
                blend_r=0.1
            )

time.sleep(10)

robot.close()




