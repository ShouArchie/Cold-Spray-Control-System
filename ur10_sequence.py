from __future__ import annotations

import math
import time
from typing import List, Sequence

import urx
from urx.urrobot import RobotException

# Params
ROBOT_IP = "192.168.10.223"  # UR10 IP address
ROT_DEG = 15.0               # ±Ry rotation magnitude (degrees)
STEP_MM = 50.0               # Y translation distance (millimetres)

# Motion parameters
ACC = 1.2  # acceleration (m/s² or rad/s²)
VEL = 0.5  # velocity (m/s or rad/s)

# Starting joint angles (degrees → radians)
START_JOINTS_DEG = [-2.66, -95, 116.5, 253.35, 270, 83.75]
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


# TCP Helpers

def set_tcp_offset(
    robot: urx.Robot,
    x_mm: float = 0.0,
    y_mm: float = 0.0,
    z_mm: float = 0.0,
    rx_deg: float = 0.0,
    ry_deg: float = 0.0,
    rz_deg: float = 0.0,
):
    x = x_mm / 1000.0
    y = y_mm / 1000.0
    z = z_mm / 1000.0
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)

    # Apply the new TCP on the controller
    robot.set_tcp((x, y, z, rx, ry, rz))
    time.sleep(0.05)

    print(
        "✓ TCP offset set to "
        f"(dx={x_mm:.1f} mm, dy={y_mm:.1f} mm, dz={z_mm:.1f} mm, "
        f"rx={rx_deg:.1f}°, ry={ry_deg:.1f}°, rz={rz_deg:.1f}°)"
    )

# --- Orientation Math Helpers -------------------------------------------------

def _aa_to_mat(rx: float, ry: float, rz: float):
    """Convert axis-angle vector to 3×3 rotation matrix."""
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        # Identity rotation
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    kx, ky, kz = rx / theta, ry / theta, rz / theta
    c = math.cos(theta)
    s = math.sin(theta)
    v = 1 - c
    return [
        [kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s],
        [ky * kx * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s],
        [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c],
    ]


def _mat_mul(a, b):
    """Matrix multiplication for 3×3 matrices."""
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _mat_to_aa(R):
    """Convert 3×3 rotation matrix to axis-angle vector."""
    trace = R[0][0] + R[1][1] + R[2][2]
    cos_theta = max(min((trace - 1.0) / 2.0, 1.0), -1.0)
    theta = math.acos(cos_theta)
    if theta < 1e-12:
        # Identity rotation
        return (0.0, 0.0, 0.0)
    sin_theta = math.sin(theta)
    rx = (R[2][1] - R[1][2]) / (2.0 * sin_theta) * theta
    ry = (R[0][2] - R[2][0]) / (2.0 * sin_theta) * theta
    rz = (R[1][0] - R[0][1]) / (2.0 * sin_theta) * theta
    return (rx, ry, rz)


def _rot_y(angle_rad: float):
    """Rotation matrix about Y axis by *angle_rad* (right-hand rule)."""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


# --- Tool-frame rotation -------------------------------------------------------

def rotate_tcp_y(robot: urx.Robot, degrees: float):
    """Rotate the tool around **its own Y (green) axis** by *degrees*.

    Unlike simply tweaking the *ry* component of the axis-angle vector (which can
    give unintuitive results), this utility performs the rotation in proper
    matrix form so the TCP **position remains unchanged** while the orientation
    is rotated about the local Y-axis.
    """
    # Current pose
    pose = get_tcp_pose(robot)
    x, y, z, rx, ry, rz = pose

    # Current orientation → matrix
    R = _aa_to_mat(rx, ry, rz)

    # Incremental rotation about local Y axis
    dR = _rot_y(math.radians(degrees))

    # Compose: R_new = R * dR (apply in tool frame)
    R_new = _mat_mul(R, dR)

    # Back to axis-angle
    rx_n, ry_n, rz_n = _mat_to_aa(R_new)

    # Send motion with unchanged translation and new orientation
    new_pose = [x, y, z, rx_n, ry_n, rz_n]
    send_movel(robot, new_pose)
    wait_until_pose(robot, new_pose)

    print(
        f"   ↳ Rotated {degrees:.1f}° about tool Y-axis (green); now Ry component = {math.degrees(ry_n):.2f}°"
    )


def translate_tcp(
    robot: urx.Robot,
    dx_mm: float = 0.0,
    dy_mm: float = 0.0,
    dz_mm: float = 0.0,
):
    """Translate the TCP along its own axes by the specified millimetres.

    Positive X = tool red axis, Y = green, Z = blue (following UR convention).
    The translation is applied in **tool coordinates**, so the TCP orientation is
    preserved, and motion feels identical to pendant "Tool → X/Y/Z" jogs.
    """
    if dx_mm == dy_mm == dz_mm == 0.0:
        return  # nothing to do

    # Current pose and orientation matrix
    pose = get_tcp_pose(robot)
    x, y, z, rx, ry, rz = pose
    R = _aa_to_mat(rx, ry, rz)

    # Delta vector in tool frame (→ metres)
    d_tool = [dx_mm / 1000.0, dy_mm / 1000.0, dz_mm / 1000.0]

    # Convert to base frame: d_base = R * d_tool
    d_base = [
        sum(R[i][j] * d_tool[j] for j in range(3)) for i in range(3)
    ]

    # New Cartesian position in base frame
    new_pos = [x + d_base[0], y + d_base[1], z + d_base[2]]

    new_pose = new_pos + [rx, ry, rz]
    send_movel(robot, new_pose)
    wait_until_pose(robot, new_pose)

    print(
        f"   ↳ Translated (tool frame) Δx={dx_mm:.1f} mm, Δy={dy_mm:.1f} mm, Δz={dz_mm:.1f} mm"
    )


def main() -> None:
    robot = urx.Robot(ROBOT_IP)
    try:
        print(f"✓ Connected to UR10 at {ROBOT_IP}")
        move_home(robot)

        # print(f"Rotating +Ry by {ROT_DEG}° …")
        rotate_tcp_y(robot, +ROT_DEG)

        # print(f"Translating -Y by {STEP_MM} mm …")
        translate_tcp(robot, dy_mm=-STEP_MM)

        # print("Rotating −Ry back …")
        rotate_tcp_y(robot, -ROT_DEG)

        print("✓ Sequence complete")
    finally:
        print("Stopping motion and closing connection …")
        stop_linear(robot)
        robot.close()







if __name__ == "__main__":
    main() 