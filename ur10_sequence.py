from __future__ import annotations

import math
import time
from typing import List, Sequence

import urx
from urx.urrobot import RobotException

# Params
ROBOT_IP = "192.168.10.223"  # UR10 IP address
ROT_DEG = 90.0               # ±Ry rotation magnitude (degrees)
STEP_MM = 50.0               # Y translation distance (millimetres)

# Motion parameters
ACC = 1.2  # acceleration (m/s² or rad/s²)
VEL = 0.5  # velocity (m/s or rad/s)

# Starting joint angles (degrees → radians)
START_JOINTS_DEG = [0, -60, 80, -110, 270, -90]
START_JOINTS = [math.radians(a) for a in START_JOINTS_DEG]

# Precision
JOINT_EPS_DEG = 1.0            # joint-angle tolerance in °
POS_EPS_MM = 0.5               # linear tolerance in mm
ORI_EPS_DEG = 2.0              # orientation tolerance in °

# Radian Conversions and values
JOINT_EPS = math.radians(JOINT_EPS_DEG)
POS_EPS = POS_EPS_MM / 1000.0
ORI_EPS = math.radians(ORI_EPS_DEG)
POLL = 0.05                    
TIMEOUT = 20.0                 

# Functions

def wait_until_joints(robot: urx.Robot, target: Sequence[float]) -> None:
    start = time.time()
    while True:
        cur = robot.getj()
        if max(abs(c - t) for c, t in zip(cur, target)) < JOINT_EPS:
            return
        if time.time() - start > TIMEOUT:
            print("⚠️  joint wait timeout; continuing")
            return
        time.sleep(POLL)


def get_tcp_pose(robot: urx.Robot) -> List[float]:
    pose = robot.getl()
    if pose is None or len(pose) != 6:
        raise RuntimeError("Invalid TCP pose from robot")
    return list(map(float, pose))


def wait_until_pose(robot: urx.Robot, target: Sequence[float]) -> None:
    start = time.time()
    def ang_err(a: float, b: float) -> float:
        diff = abs(a - b) % (2 * math.pi)
        return diff if diff <= math.pi else (2 * math.pi - diff)

    while True:
        cur = get_tcp_pose(robot)
        pos_err = max(abs(c - t) for c, t in zip(cur[:3], target[:3]))
        ori_err = max(ang_err(c, t) for c, t in zip(cur[3:], target[3:]))
        if pos_err < POS_EPS and ori_err < ORI_EPS:
            return
        if time.time() - start > TIMEOUT:
            print("⚠️  pose wait timeout; continuing")
            return
        time.sleep(POLL)


def send_movel(robot: urx.Robot, pose: Sequence[float]):
    pose_str = ", ".join(f"{v:.6f}" for v in pose)
    robot.send_program(f"movel(p[{pose_str}], a={ACC}, v={VEL})")


def stop_linear(robot: urx.Robot):
    try:
        robot.send_program(f"stopl(a={ACC})")
    except Exception:
        pass  # ignore if already stopped


# Motion Helpers

def move_home(robot: urx.Robot):
    print("Moving to home joints …")
    try:
        robot.movej(START_JOINTS, acc=ACC, vel=VEL, wait=False)
    except RobotException as e:
        if "Robot stopped" not in str(e):
            raise
    wait_until_joints(robot, START_JOINTS)


def rotate_ry(robot: urx.Robot, degrees: float):
    pose = get_tcp_pose(robot)
    pose[4] += math.radians(degrees)
    send_movel(robot, pose)
    wait_until_pose(robot, pose)
    # Debug output
    actual = get_tcp_pose(robot)
    print(
        f"   ↳ Reached Ry = {math.degrees(actual[4]):.2f}° (target {math.degrees(pose[4]):.2f}°)"
    )


def translate_y(robot: urx.Robot, mm: float):
    pose = get_tcp_pose(robot)
    pose[1] += mm / 1000.0  # convert mm → metres
    send_movel(robot, pose)
    wait_until_pose(robot, pose)
    # Debug output
    actual = get_tcp_pose(robot)
    print(
        f"   ↳ Reached Y = {actual[1]*1000:.1f} mm (target {pose[1]*1000:.1f} mm)"
    )


# Main

def main() -> None:
    robot = urx.Robot(ROBOT_IP)
    try:
        print(f"✓ Connected to UR10 at {ROBOT_IP}")
        move_home(robot)

        print(f"Rotating +Ry by {ROT_DEG}° …")
        rotate_ry(robot, +ROT_DEG)

        print(f"Translating -Y by {STEP_MM} mm …")
        translate_y(robot, -STEP_MM)

        print("Rotating −Ry back …")
        rotate_ry(robot, -ROT_DEG)

        print("✓ Sequence complete")
    finally:
        print("Stopping motion and closing connection …")
        stop_linear(robot)
        robot.close()


if __name__ == "__main__":
    main() 