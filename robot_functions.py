from __future__ import annotations

import math
import time
from typing import List, Sequence

import urx
from urx.urrobot import RobotException

# Constants and Configuration
JOINT_EPS_DEG = 1.0            # joint-angle tolerance in °
POS_EPS_MM = 1               # linear tolerance in mm
ORI_EPS_DEG = 0.1              # orientation tolerance in °

# Converted constants
JOINT_EPS = math.radians(JOINT_EPS_DEG)
POS_EPS = POS_EPS_MM / 1000.0
ORI_EPS = math.radians(ORI_EPS_DEG)
POLL = 0.005                    
TIMEOUT = 60.0                 

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

    print(
        f"   ↳ Translated (tool frame) Δx={dx_mm:.1f} mm, Δy={dy_mm:.1f} mm, Δz={dz_mm:.1f} mm"
    )

# -----------------------------------------------------------------------------
# Complex Paths – Cone Sweeping
# -----------------------------------------------------------------------------


def conical_motion(
    robot: urx.Robot,
    tilt_deg: float = 20.0,
    revolutions: float = 1.0,
    steps: int = 72,
    acc: float = 1.0,
    vel: float = 0.3,
    debug: bool = False,
    avoid_singular: bool = True,
    sing_tol_deg: float = 15.0,
    blend_mm: float = 0.0,
):
    """Sweep the tool orientation along a conical surface while **keeping the TCP
    position fixed** and forcing the tool Z-axis (blue) to always point roughly
    downward (−Z in base frame).

    The red axis (tool X) traces the generatrix of a cone whose axis is the
    base −X direction.
    """

    # Capture current TCP position (we'll keep translation fixed)
    pose = get_tcp_pose(robot)
    x, y, z, *_ = pose

    axis = (-1.0, 0.0, 0.0)  # cone axis (towards −X)
    u = (0.0, 0.0, 1.0)      # basis vector ⟂ axis
    v = (0.0, 1.0, 0.0)      # second basis ⟂ axis

    theta = math.radians(tilt_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    for i in range(steps + 1):
        phi = 2 * math.pi * revolutions * i / steps
        cp, sp = math.cos(phi), math.sin(phi)

        # Direction of tool X (red) on the cone surface
        X_vec = [
            cos_t * axis[0] + sin_t * (cp * u[0] + sp * v[0]),
            cos_t * axis[1] + sin_t * (cp * u[1] + sp * v[1]),
            cos_t * axis[2] + sin_t * (cp * u[2] + sp * v[2]),
        ]
        # Normalize (for numerical stability)
        mag = math.sqrt(sum(c * c for c in X_vec))
        X_vec = [c / mag for c in X_vec]

        # Aim Z axis roughly downward: pick Y = normalize(Z_down × X)
        Z_down = (0.0, 0.0, -1.0)
        Y_vec = [
            Z_down[1] * X_vec[2] - Z_down[2] * X_vec[1],
            Z_down[2] * X_vec[0] - Z_down[0] * X_vec[2],
            Z_down[0] * X_vec[1] - Z_down[1] * X_vec[0],
        ]
        mag_y = math.sqrt(sum(c * c for c in Y_vec))
        if mag_y < 1e-8:
            # fallback: pick arbitrary perpendicular if cross-product degenerate
            Y_vec = [0.0, 1.0, 0.0]
            mag_y = 1.0
        Y_vec = [c / mag_y for c in Y_vec]

        # Recompute Z to ensure orthonormality (X × Y)
        Z_vec = [
            X_vec[1] * Y_vec[2] - X_vec[2] * Y_vec[1],
            X_vec[2] * Y_vec[0] - X_vec[0] * Y_vec[2],
            X_vec[0] * Y_vec[1] - X_vec[1] * Y_vec[0],
        ]

        # Assemble rotation matrix columns (world frame)
        R = [
            [X_vec[0], Y_vec[0], Z_vec[0]],
            [X_vec[1], Y_vec[1], Z_vec[1]],
            [X_vec[2], Y_vec[2], Z_vec[2]],
        ]

        # Convert to axis-angle
        rx_t, ry_t, rz_t = _mat_to_aa(R)
        target_pose = [x, y, z, rx_t, ry_t, rz_t]
        # Skip azimuths very close to 90° or 270° if requested
        if avoid_singular:
            ang_deg = math.degrees(phi) % 360

            def ang_dist(a, b):
                """Shortest distance between angles a and b (deg) on a circle."""
                d = (a - b + 180) % 360 - 180
                return abs(d)

            if min(ang_dist(ang_deg, 90), ang_dist(ang_deg, 270)) < sing_tol_deg:
                if debug:
                    print(
                        f"  Step {i:>3}/{steps}: phi={ang_deg:.1f}°  — skipped (singularity)"
                    )
                continue

        # Use joint-interpolated move to keep solution near current joints
        send_movej_pose(robot, target_pose, acc, vel, blend_mm=blend_mm)
        wait_until_pose(robot, target_pose)

        if debug:
            joints = robot.getj()
            j_deg = [f"{math.degrees(a):.1f}" for a in joints]
            print(f"  Step {i:>3}/{steps}: phi={math.degrees(phi):.1f}°  joints={j_deg}")

    print(
        f"✓ Completed conical sweep with Z-axis down: tilt={tilt_deg}°, revolutions={revolutions}, steps={steps}"
    )


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

    print(
        f"   ↳ Rotated ΔRx={rx_deg:.1f}°, ΔRy={ry_deg:.1f}°, ΔRz={rz_deg:.1f}° (tool frame)"
    )

# -----------------------------------------------------------------------------
# Combined tilt + TCP square + spin helper
# -----------------------------------------------------------------------------

def tilt_and_spin(
    robot: urx.Robot,
    tilt_deg: float = 10.0,
    revolutions: float = 1.0,
    steps: int = 120,
    tcp_x_mm: float = -257.81,
    tcp_y_mm: float = 0.0,
    tcp_z_mm: float = 60.3,
    tcp_rx_deg: float = 0.0,
    tcp_ry_deg: float = 0.0,
    tcp_rz_deg: float = 0.0,
    acc: float = 0.5,
    vel: float = 0.5,
):
    """Tilt the tool down by *tilt_deg*, square the TCP, then spin about X.

    1. Rotate around tool Y by ``-tilt_deg`` (tool tips down).
    2. Add ``+tilt_deg`` to the TCP Ry component so the controller again sees
       a *level* tool frame.
    3. Incrementally rotate the tool about its own X-axis, completing the
       requested number of revolutions.

    Parameters
    ----------
    robot : urx.Robot
    tilt_deg : float
        Downward tilt angle (positive values tilt toward −Z).
    revolutions : float
        How many full 360° turns to perform.
    steps : int
        Number of incremental X-axis rotations (higher → smoother).
    tcp_* : float
        The TCP offset (in millimetres / degrees) *before* tilting. Only Ry is
        modified internally; the others are kept as supplied.
    acc, vel : float
        Motion parameters for the incremental moves.
    """

    if steps <= 0:
        raise ValueError("steps must be positive")

    # 1) Tilt the tool down
    rotate_tcp_y(robot, degrees=-tilt_deg, acc=acc, vel=vel)

    # 2) Square the TCP by adding +tilt_deg to Ry component
    rf_ry = tcp_ry_deg + tilt_deg  # new Ry in degrees
    set_tcp_offset(
        robot,
        x_mm=tcp_x_mm,
        y_mm=tcp_y_mm,
        z_mm=tcp_z_mm,
        rx_deg=tcp_rx_deg,
        ry_deg=rf_ry,
        rz_deg=tcp_rz_deg,
    )

    # 3) Spin about tool X in small increments
    delta = 360.0 * revolutions / steps
    for _ in range(steps):
        rotate_tcp_x(robot, degrees=delta, acc=acc, vel=vel)

    print(
        f"✓ Completed tilt_and_spin: tilt={tilt_deg}°, revolutions={revolutions}, steps={steps}"
    )

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
# URScript program generation with movep (smooth p-curve)
# -----------------------------------------------------------------------------

def conical_motion_movep_script(
    robot: urx.Robot,
    tilt_deg: float = 20.0,
    revolutions: float = 1.0,
    steps: int = 72,
    acc: float = 1.0,
    vel: float = 0.3,
    blend_mm: float = 5.0,
    avoid_singular: bool = True,
    sing_tol_deg: float = 2.0,
):
    """Generate and upload a URScript that executes the conical sweep using
    movep / movep_add_waypoint for high-smoothness (requires UR SW ≥ 5.10 or any
    e-Series controller).
    """

    x0, y0, z0, *_ = get_tcp_pose(robot)

    axis = (-1.0, 0.0, 0.0)
    u = (0.0, 0.0, 1.0)
    v = (0.0, 1.0, 0.0)
    theta = math.radians(tilt_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)

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
        R = [[X[0], Y[0], Z[0]],[X[1], Y[1], Z[1]],[X[2], Y[2], Z[2]]]
        rx, ry, rz = _mat_to_aa(R)
        pts.append([x0, y0, z0, rx, ry, rz])

    if len(pts) < 3:
        raise RuntimeError("movep path needs at least 3 waypoints after skipping singularities")

    r_m = max(0.0, blend_mm) / 1000.0
    first_pose = ", ".join(f"{v:.6f}" for v in pts[0])

    lines = ["def cone_path():"]
    lines.append(f"  movep(p[{first_pose}], a={acc}, v={vel}, r={r_m:.4f})")
    for p in pts[1:]:
        pose_str = ", ".join(f"{v:.6f}" for v in p)
        lines.append(f"  movep_add_waypoint(p[{pose_str}])")
    lines.append("end")
    lines.append("cone_path()")

    send_urscript(robot, "\n".join(lines))