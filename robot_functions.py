from __future__ import annotations

import math
import time
from typing import List, Sequence

import urx
from urx.urrobot import RobotException

# Constants and Configuration
JOINT_EPS_DEG = 0.05           # joint-angle tolerance in °
POS_EPS_MM = 1               # linear tolerance in mm
ORI_EPS_DEG = 0.1              # orientation tolerance in °

# Converted constants
JOINT_EPS = math.radians(JOINT_EPS_DEG)
POS_EPS = POS_EPS_MM / 1000.0
ORI_EPS = math.radians(ORI_EPS_DEG)
POLL = 0.005                    
TIMEOUT = 180                 

# -----------------------------------------------------------------------------
# Robot Connection and Basic Operations
# -----------------------------------------------------------------------------

def connect_robot(ip: str) -> urx.Robot:
    """Connect to UR robot at specified IP address."""
    robot = urx.Robot(ip)
    print(f"✓ Connected to UR10 at {ip}")
    return robot

def disconnect_robot(robot: urx.Robot):
    """Safely disconnect from robot."""
    try:
        stop_linear(robot)
        robot.close()
        print("✓ Robot connection closed")
    except Exception as e:
        print(f"⚠️ Error during disconnect: {e}")

# -----------------------------------------------------------------------------
# Motion Control and Waiting Functions
# -----------------------------------------------------------------------------

def wait_until_joints(robot: urx.Robot, target: Sequence[float]) -> None:
    """Wait until robot reaches target joint configuration."""
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
    """Get current TCP pose [x, y, z, rx, ry, rz]."""
    pose = robot.getl()
    if pose is None or len(pose) != 6:
        raise RuntimeError("Invalid TCP pose from robot")
    return list(map(float, pose))

def wait_until_pose(robot: urx.Robot, target: Sequence[float]) -> None:
    """Wait until robot reaches target TCP pose."""
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

def send_movel(
    robot: urx.Robot,
    pose: Sequence[float],
    acc: float = 1.2,
    vel: float = 0.5,
    blend_mm: float = 0.0,
):
    """Send movel command with optional blend radius (mm)."""
    pose_str = ", ".join(f"{v:.6f}" for v in pose)
    r_part = f", r={blend_mm/1000.0:.4f}" if blend_mm > 0 else ""
    robot.send_program(f"movel(p[{pose_str}], a={acc}, v={vel}{r_part})")


# New helper: joint-interpolated move to pose (lets robot choose minimal joint path)

def send_movej_pose(
    robot: urx.Robot,
    pose: Sequence[float],
    acc: float = 1.2,
    vel: float = 0.5,
    blend_mm: float = 0.0,
):
    """Send movej command targeting a Cartesian pose (UR will IK to joints) with optional blend radius."""
    pose_str = ", ".join(f"{v:.6f}" for v in pose)
    r_part = f", r={blend_mm/1000.0:.4f}" if blend_mm > 0 else ""
    robot.send_program(f"movej(p[{pose_str}], a={acc}, v={vel}{r_part})")

def stop_linear(robot: urx.Robot, acc: float = 1.2):
    """Stop linear motion."""
    try:
        robot.send_program(f"stopl(a={acc})")
    except Exception:
        pass  # ignore if already stopped

def move_to_joint_position(robot: urx.Robot, joints: Sequence[float], acc: float = 1.2, vel: float = 0.5, wait: bool = True):
    """Move robot to specified joint configuration."""
    print("Moving to target joint position …")
    try:
        robot.movej(joints, acc=acc, vel=vel, wait=False)
    except RobotException as e:
        if "Robot stopped" not in str(e):
            raise
    if wait:
        wait_until_joints(robot, joints)

def send_urscript(robot: urx.Robot, script: str):
    """Send raw URScript to robot."""
    print("▶ Sending URScript program …")
    robot.send_program(script)
    print("✓ Program sent")

# -----------------------------------------------------------------------------
# TCP Configuration
# -----------------------------------------------------------------------------

def set_tcp_offset(
    robot: urx.Robot,
    x_mm: float = 0.0,
    y_mm: float = 0.0,
    z_mm: float = 0.0,
    rx_deg: float = 0.0,
    ry_deg: float = 0.0,
    rz_deg: float = 0.0,
):
    """Set TCP offset in millimeters and degrees."""
    x = x_mm / 1000.0
    y = y_mm / 1000.0
    z = z_mm / 1000.0
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)

    robot.set_tcp((x, y, z, rx, ry, rz))
    time.sleep(0.05)

    print(
        "✓ TCP offset set to "
        f"(dx={x_mm:.1f} mm, dy={y_mm:.1f} mm, dz={z_mm:.1f} mm, "
        f"rx={rx_deg:.1f}°, ry={ry_deg:.1f}°, rz={rz_deg:.1f}°)"
    )

# -----------------------------------------------------------------------------
# Orientation Math Helpers
# -----------------------------------------------------------------------------

def _aa_to_mat(rx: float, ry: float, rz: float):
    """Convert axis-angle vector to 3×3 rotation matrix."""
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
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
        return (0.0, 0.0, 0.0)
    sin_theta = math.sin(theta)
    rx = (R[2][1] - R[1][2]) / (2.0 * sin_theta) * theta
    ry = (R[0][2] - R[2][0]) / (2.0 * sin_theta) * theta
    rz = (R[1][0] - R[0][1]) / (2.0 * sin_theta) * theta
    return (rx, ry, rz)

def _rot_y(angle_rad: float):
    """Rotation matrix about Y axis by angle_rad (right-hand rule)."""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]

def _rot_z(angle_rad: float):
    """Rotation matrix about Z axis by angle_rad (right-hand rule)."""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]

# Add new rotation helper about X axis

def _rot_x(angle_rad: float):
    """Rotation matrix about X axis by angle_rad (right-hand rule)."""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]

# -----------------------------------------------------------------------------
# Advanced TCP Motion Functions
# -----------------------------------------------------------------------------

def rotate_tcp_y(robot: urx.Robot, degrees: float, acc: float = 1.2, vel: float = 0.5):
    """Rotate the tool around its own Y (green) axis by degrees."""
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
    send_movel(robot, new_pose, acc, vel)
    wait_until_pose(robot, new_pose)

def rotate_tcp_z(robot: urx.Robot, degrees: float, acc: float = 1.2, vel: float = 0.5):
    """Rotate the tool around its own Z (blue) axis by degrees."""
    pose = get_tcp_pose(robot)
    x, y, z, rx, ry, rz = pose

    # Current orientation → matrix
    R = _aa_to_mat(rx, ry, rz)

    # Incremental rotation about local Z axis
    dR = _rot_z(math.radians(degrees))

    # Compose: R_new = R * dR (apply in tool frame)
    R_new = _mat_mul(R, dR)

    # Back to axis-angle
    rx_n, ry_n, rz_n = _mat_to_aa(R_new)

    # Send motion with unchanged translation and new orientation
    new_pose = [x, y, z, rx_n, ry_n, rz_n]
    send_movel(robot, new_pose, acc, vel)
    wait_until_pose(robot, new_pose)

    print(
        f"   ↳ Rotated {degrees:.1f}° about tool Y-axis (green); now Ry component = {math.degrees(ry_n):.2f}°"
    )

def rotate_tcp_x(robot: urx.Robot, degrees: float, acc: float = 1.2, vel: float = 0.5):
    """Rotate the tool around its own X (red) axis by degrees."""
    pose = get_tcp_pose(robot)
    x, y, z, rx, ry, rz = pose

    # Current orientation → matrix
    R = _aa_to_mat(rx, ry, rz)

    # Incremental rotation about local X axis
    dR = _rot_x(math.radians(degrees))

    # Compose: R_new = R * dR (apply in tool frame)
    R_new = _mat_mul(R, dR)

    # Back to axis-angle
    rx_n, ry_n, rz_n = _mat_to_aa(R_new)

    # Send motion with unchanged translation and new orientation
    new_pose = [x, y, z, rx_n, ry_n, rz_n]
    send_movel(robot, new_pose, acc, vel)
    wait_until_pose(robot, new_pose)

    print(
        f"   ↳ Rotated {degrees:.1f}° about tool X-axis (red); now Rx component = {math.degrees(rx_n):.2f}°"
    )

def translate_tcp(
    robot: urx.Robot,
    dx_mm: float = 0.0,
    dy_mm: float = 0.0,
    dz_mm: float = 0.0,
    acc: float = 1.2,
    vel: float = 0.5
):
    """Translate the TCP along its own axes by the specified millimeters."""
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
    send_movel(robot, new_pose, acc, vel)
    wait_until_pose(robot, new_pose)

    # print(
    #     f"   ↳ Translated (tool frame) Δx={dx_mm:.1f} mm, Δy={dy_mm:.1f} mm, Δz={dz_mm:.1f} mm"
    # )

# -----------------------------------------------------------------------------
# Generic incremental rotation (all three axes)
# -----------------------------------------------------------------------------

def rotate_tcp(
    robot: urx.Robot,
    rx_deg: float = 0.0,
    ry_deg: float = 0.0,
    rz_deg: float = 0.0,
    acc: float = 1.2,
    vel: float = 0.5,
):
    """Incrementally rotate the TCP about its own X, Y and Z axes.

    The rotations are applied in **X → Y → Z** order, all in the TOOL frame,
    and the TCP translation is unchanged.
    """
    if rx_deg == ry_deg == rz_deg == 0.0:
        return  # nothing to do

    pose = get_tcp_pose(robot)
    x, y, z, rx, ry, rz = pose
    R = _aa_to_mat(rx, ry, rz)

    dR_x = _rot_x(math.radians(rx_deg))
    dR_y = _rot_y(math.radians(ry_deg))
    dR_z = _rot_z(math.radians(rz_deg))

    # Apply increments in sequence (tool-frame)
    R_new = _mat_mul(_mat_mul(R, dR_x), _mat_mul(dR_y, dR_z))

    rx_n, ry_n, rz_n = _mat_to_aa(R_new)
    new_pose = [x, y, z, rx_n, ry_n, rz_n]
    send_movel(robot, new_pose, acc, vel)
    wait_until_pose(robot, new_pose)

    # print(
    #     f"   ↳ Rotated ΔRx={rx_deg:.1f}°, ΔRy={ry_deg:.1f}°, ΔRz={rz_deg:.1f}° (tool frame)"
    # )
# -----------------------------------------------------------------------------
# URScript program generation for conical sweep
# -----------------------------------------------------------------------------

def conical_motion_script(
    robot: urx.Robot,
    tilt_deg: float = 20.0,
    revolutions: float = 1.0,
    steps: int = 72,
    acc: float = 0.1,
    vel: float = 0.1,
    blend_mm: float = 1.0,
    avoid_singular: bool = True,
    sing_tol_deg: float = 2.0,
):
    """Generate and send a single URScript program that performs the conical
    sweep with constant blend radius.

    This is useful when you want the robot to execute the whole path natively
    (smooth blending, no round-trip latency).
    """

    x0, y0, z0, *_ = get_tcp_pose(robot)

    axis = (-1.0, 0.0, 0.0)
    u = (0.0, 0.0, 1.0)
    v = (0.0, 1.0, 0.0)
    theta = math.radians(tilt_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    blend_m = max(0.0, blend_mm) / 1000.0
    pts = []

    for i in range(steps + 1):
        phi = 2 * math.pi * revolutions * i / steps
        ang = math.degrees(phi) % 360
        if avoid_singular and min(abs(((ang - 90 + 180) % 360) - 180), abs(((ang - 270 + 180) % 360) - 180)) < sing_tol_deg:
            continue

        cp, sp = math.cos(phi), math.sin(phi)
        X = [cos_t*axis[0] + sin_t*(cp*u[0] + sp*v[0]),
             cos_t*axis[1] + sin_t*(cp*u[1] + sp*v[1]),
             cos_t*axis[2] + sin_t*(cp*u[2] + sp*v[2])]
        mag = math.sqrt(sum(c*c for c in X)) or 1.0
        X = [c/mag for c in X]

        Zdown = (0.0, 0.0, -1.0)
        Y = [Zdown[1]*X[2]-Zdown[2]*X[1], Zdown[2]*X[0]-Zdown[0]*X[2], Zdown[0]*X[1]-Zdown[1]*X[0]]
        mag_y = math.sqrt(sum(c*c for c in Y)) or 1.0
        Y = [c/mag_y for c in Y]
        Z = [X[1]*Y[2]-X[2]*Y[1], X[2]*Y[0]-X[0]*Y[2], X[0]*Y[1]-X[1]*Y[0]]
        R = [[X[0], Y[0], Z[0]], [X[1], Y[1], Z[1]], [X[2], Y[2], Z[2]]]
        rx, ry, rz = _mat_to_aa(R)
        pts.append([x0, y0, z0, rx, ry, rz])

    lines = ["def cone_path():"]
    prev = None
    for idx, p in enumerate(pts):
        pose_str = ", ".join(f"{v:.6f}" for v in p)
        if idx == len(pts) - 1 or blend_m == 0.0:
            # Last point or no blending requested
            lines.append(f"  movej(p[{pose_str}], a={acc}, v={vel})")
        else:
            # Limit blend radius to < half distance to next waypoint (UR requirement)
            if prev is None:
                prev = p
            dist = math.sqrt(sum((p[i]-prev[i])**2 for i in range(3)))  # metres
            max_r = 0.45 * dist  # leave some margin
            r_use = min(blend_m, max_r)
            if r_use < 1e-6:
                lines.append(f"  movej(p[{pose_str}], a={acc}, v={vel})")
            else:
                lines.append(f"  movej(p[{pose_str}], a={acc}, v={vel}, r={r_use:.4f})")
            prev = p
    lines.append("end")
    lines.append("cone_path()")

    send_urscript(robot, "\n".join(lines))

# -----------------------------------------------------------------------------
# URScript program generation with servoJ for smooth conical sweep
# -----------------------------------------------------------------------------

def conical_motion_servoj_script(
    robot: urx.Robot,
    tilt_deg: float = 20.0,
    revolutions: float = 1.0,
    steps: int = 720,
    cycle_s: float = 0.008,
    lookahead_time: float = 0.1,
    gain: int = 300,
    avoid_singular: bool = True,
    sing_tol_deg: float = 1.0,
):

    # Current TCP position (XYZ only) becomes the apex of the cone
    x0, y0, z0, *_ = get_tcp_pose(robot)

    # Unit vectors for frame construction (same math as *conical_motion_script*)
    axis = (-1.0, 0.0, 0.0)  # desired mean tool axis (−X in base)
    u = (0.0, 0.0, 1.0)      # helper vectors to spin around *axis*
    v = (0.0, 1.0, 0.0)

    theta = math.radians(tilt_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    # Build full list of poses (axis-angle) that realise the cone
    pts: list[list[float]] = []
    for i in range(steps + 1):
        phi = 2 * math.pi * revolutions * i / steps
        ang = math.degrees(phi) % 360
        if avoid_singular and min(
            abs(((ang - 90 + 180) % 360) - 180),
            abs(((ang - 270 + 180) % 360) - 180),
        ) < sing_tol_deg:
            # Skip configurations that get too close to wrist singularities
            continue

        cp, sp = math.cos(phi), math.sin(phi)
        X = [
            cos_t * axis[0] + sin_t * (cp * u[0] + sp * v[0]),
            cos_t * axis[1] + sin_t * (cp * u[1] + sp * v[1]),
            cos_t * axis[2] + sin_t * (cp * u[2] + sp * v[2]),
        ]
        mag = math.sqrt(sum(c * c for c in X)) or 1.0
        X = [c / mag for c in X]

        # Orthonormal Y,Z to complete the frame (Z = X × Y, Y chosen so Z≈world −Z)
        Zdown = (0.0, 0.0, -1.0)
        Y = [
            Zdown[1] * X[2] - Zdown[2] * X[1],
            Zdown[2] * X[0] - Zdown[0] * X[2],
            Zdown[0] * X[1] - Zdown[1] * X[0],
        ]
        mag_y = math.sqrt(sum(c * c for c in Y)) or 1.0
        Y = [c / mag_y for c in Y]
        Z = [X[1] * Y[2] - X[2] * Y[1], X[2] * Y[0] - X[0] * Y[2], X[0] * Y[1] - X[1] * Y[0]]
        R = [[X[0], Y[0], Z[0]], [X[1], Y[1], Z[1]], [X[2], Y[2], Z[2]]]
        rx, ry, rz = _mat_to_aa(R)
        pts.append([x0, y0, z0, rx, ry, rz])

    # Assemble URScript program
    lines = ["def cone_servoj():"]
    for p in pts:
        pose_str = ", ".join(f"{v:.6f}" for v in p)
        lines.append(
            f"  servoj(get_inverse_kin(p[{pose_str}]), t={cycle_s}, lookahead_time={lookahead_time}, gain={gain})"
        )
        lines.append("  sync()")
    lines.append("end")
    lines.append("cone_servoj()")

    # Send program for execution
    send_urscript(robot, "\n".join(lines))


def wait_until_idle(robot: urx.Robot, eps_rad: float = 0.0005, stable_time: float = 0.15, poll: float = 0.002, timeout: float = TIMEOUT) -> None:
    """Block until robot has stopped moving (all joints stable).

    This method is controller-agnostic: it simply monitors successive joint
    positions and waits until they change by < *eps_rad* for at least
    *stable_time* seconds.  Useful when secmon/program flags are unreliable.
    """
    start = time.time()
    last = robot.getj()
    stable_start = None
    while True:
        cur = robot.getj()
        if max(abs(c - l) for c, l in zip(cur, last)) < eps_rad:
            # joints have barely moved since last sample
            if stable_start is None:
                stable_start = time.time()
            elif time.time() - stable_start >= stable_time:
                return  # stationary long enough
        else:
            stable_start = None  # movement resumed
        if time.time() - start > timeout:
            print("⚠️  idle wait timeout; continuing")
            return
        last = cur
        time.sleep(poll)
