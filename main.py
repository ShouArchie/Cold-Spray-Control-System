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

def zigzag():
    HOMETest = [math.radians(a) for a in [201.64, -54.86, 109.85, 214.57, 269.77, 111.69]]
    try:
        print(f"✓ Connected to UR10 at {ROBOT_IP}")
        # Starting Pose
        rf.move_to_joint_position(robot, HOMETest, acc=1, vel=0.5)

        # First Plate Pose
        rf.move_to_joint_position(robot, HOMETest, acc=1, vel=0.5)
        print("First Plate Pose")
        plate_pose = rf.get_tcp_pose(robot)
        ACCEL = 1.2
        VEL = 0.25
        # Cold Spray First Plate using onebyonesnake function:
        urscript_code = paths.onebyonesnake(acc=ACCEL, vel=VEL, blend_r=0.0001)
        robot.send_program(urscript_code)

        time.sleep(2)
        
        target_pose = plate_pose.copy()
        target_pose[1] = target_pose[1] - (0.0274 * 2)  # 0.0274 * 2 = 0.0548
        rf.wait_until_pose(robot, target_pose)
        print("Complete")

        rf.move_to_joint_position(robot, HOMETest, acc=0.1, vel=0.1)


    finally:
        print("Stopping motion and closing connection …")
        rf.stop_linear(robot)
        robot.close()



robot = urx.Robot(ROBOT_IP)



