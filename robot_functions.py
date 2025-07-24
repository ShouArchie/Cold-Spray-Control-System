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

def send_movel(robot: urx.Robot, pose: Sequence[float], acc: float = 1.2, vel: float = 0.5):
    """Send movel command to robot."""
    pose_str = ", ".join(f"{v:.6f}" for v in pose)
    robot.send_program(f"movel(p[{pose_str}], a={acc}, v={vel})")

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

def circular_motion_fixed_tcp(
    robot: urx.Robot, 
    start_angle: float = -30.0,
    end_angle: float = 30.0, 
    num_points: int = 20,
    tcp_offset_z: float = 60.3,  # Distance from flange to TCP in mm
    acc: float = 0.5, 
    vel: float = 0.1
):
    """
    Create circular motion where TCP stays fixed but flange moves in circular arc.
    
    The flange traces a circular path in the X-Z plane while TCP remains stationary.
    This is achieved by rotating around the tool's Y-axis while compensating 
    the flange position to keep TCP fixed.
    
    Args:
        robot: UR robot instance
        start_angle: Starting Y-axis rotation angle in degrees
        end_angle: Ending Y-axis rotation angle in degrees  
        num_points: Number of points in the circular path
        tcp_offset_z: Distance from flange to TCP along tool Z-axis in mm
        acc: Acceleration for movement
        vel: Velocity for movement
    """
    # Get current pose
    current_pose = get_tcp_pose(robot)
    tcp_x, tcp_y, tcp_z = current_pose[:3]
    
    print(f"Starting circular motion: TCP fixed at [{tcp_x:.3f}, {tcp_y:.3f}, {tcp_z:.3f}]")
    print(f"Angle range: {start_angle}° to {end_angle}°, {num_points} points")
    
    # Calculate angle increment
    angle_step = (end_angle - start_angle) / (num_points - 1)
    
    for i in range(num_points):
        # Current angle
        current_angle = start_angle + i * angle_step
        angle_rad = math.radians(current_angle)
        
        # Calculate flange position to keep TCP fixed
        # Y-axis rotation causes flange to move in X-Z plane (circular arc)
        offset_z_m = tcp_offset_z / 1000.0  # Convert mm to meters
        
        # Flange offset from TCP position
        flange_x = tcp_x + offset_z_m * math.sin(angle_rad)
        flange_y = tcp_y  # Y doesn't change for Y-axis rotation
        flange_z = tcp_z - offset_z_m * math.cos(angle_rad)
        
        # Create rotation matrix for current angle
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        rot_y_matrix = [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
        
        # Convert to axis-angle representation
        # For pure Y rotation: rx=0, ry=angle, rz=0
        rx_new = 0.0
        ry_new = angle_rad  
        rz_new = 0.0
        
        # Create target pose
        target_pose = [flange_x, flange_y, flange_z, rx_new, ry_new, rz_new]
        
        # Move to position
        send_movel(robot, target_pose, acc, vel)
        wait_until_pose(robot, target_pose)
        
        print(f"   Point {i+1}/{num_points}: Angle={current_angle:.1f}°, Flange=[{flange_x:.3f}, {flange_y:.3f}, {flange_z:.3f}]")
    
    print("Circular motion complete!")

def conical_motion_fixed_tcp(
    robot: urx.Robot,
    max_angle: float = 30.0,
    num_points: int = 20,
    tcp_offset_x: float = -257.81,  # X offset from flange to TCP in mm
    tcp_offset_y: float = 0.0,      # Y offset from flange to TCP in mm  
    tcp_offset_z: float = 60.3,     # Z offset from flange to TCP in mm
    acc: float = 0.5,
    vel: float = 0.1,
    blend_r: float = 0.01  # Blend radius for smooth motion
):
    """
    Create conical motion where TCP stays fixed but tool traces outer surface of cone.
    
    The tool orientation rotates in a circle (X and Y axes), tracing the surface of a cone
    while keeping the TCP position completely stationary. The flange moves in a complex
    3D path to maintain the fixed TCP.
    
    Motion pattern (one full rotation):
    - Start: Y=-max_angle, X=0
    - Quarter: Y=0, X=-max_angle  
    - Half: Y=+max_angle, X=0
    - Three-quarter: Y=0, X=+max_angle
    - End: Y=-max_angle, X=0
    
    Args:
        robot: UR robot instance
        max_angle: Maximum tilt angle in degrees (cone half-angle)
        num_points: Number of points in the conical path
        tcp_offset_x: X offset from flange to TCP in mm
        tcp_offset_y: Y offset from flange to TCP in mm
        tcp_offset_z: Z offset from flange to TCP in mm
        acc: Acceleration for movement
        vel: Velocity for movement
        blend_r: Blend radius for smooth continuous motion
    """
    # Get current pose
    current_pose = get_tcp_pose(robot)
    tcp_x, tcp_y, tcp_z = current_pose[:3]
    base_rx, base_ry, base_rz = current_pose[3:]
    
    print(f"Starting conical motion: TCP fixed at [{tcp_x:.3f}, {tcp_y:.3f}, {tcp_z:.3f}]")
    print(f"Cone half-angle: {max_angle}°, {num_points} points")
    
    # Convert TCP offsets to meters
    offset_x_m = tcp_offset_x / 1000.0
    offset_y_m = tcp_offset_y / 1000.0
    offset_z_m = tcp_offset_z / 1000.0
    
    for i in range(num_points + 1):  # +1 to complete the circle
        # Calculate circular parameter (0 to 2π)
        t = 2 * math.pi * i / num_points
        
        # Calculate X and Y rotation angles in a circular pattern
        angle_rad = math.radians(max_angle)
        ry_current = angle_rad * math.cos(t)  # Y-axis rotation
        rx_current = angle_rad * math.sin(t)  # X-axis rotation
        
        # Create rotation matrices for X and Y rotations
        # X rotation matrix
        cx = math.cos(rx_current)
        sx = math.sin(rx_current)
        rot_x = [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]]
        
        # Y rotation matrix  
        cy = math.cos(ry_current)
        sy = math.sin(ry_current)
        rot_y = [[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]]
        
        # Combined rotation: R = Ry * Rx (apply X rotation first, then Y)
        combined_rot = _mat_mul(rot_y, rot_x)
        
        # Calculate where flange needs to be to keep TCP fixed
        # TCP = Flange + R * [offset_x, offset_y, offset_z]
        # So: Flange = TCP - R * [offset_x, offset_y, offset_z]
        
        # Tool vector in rotated frame (apply rotation matrix to tool offset)
        tool_vector = [
            combined_rot[0][0] * offset_x_m + combined_rot[0][1] * offset_y_m + combined_rot[0][2] * offset_z_m,
            combined_rot[1][0] * offset_x_m + combined_rot[1][1] * offset_y_m + combined_rot[1][2] * offset_z_m,
            combined_rot[2][0] * offset_x_m + combined_rot[2][1] * offset_y_m + combined_rot[2][2] * offset_z_m
        ]
        
        # Calculate flange position
        flange_x = tcp_x - tool_vector[0]
        flange_y = tcp_y - tool_vector[1] 
        flange_z = tcp_z - tool_vector[2]
        
        # Convert rotation matrix back to axis-angle
        rx_new, ry_new, rz_new = _mat_to_aa(combined_rot)
        
        # Add base rotations to maintain original orientation offset
        rx_final = base_rx + rx_new
        ry_final = base_ry + ry_new  
        rz_final = base_rz + rz_new
        
        # Create target pose
        target_pose = [flange_x, flange_y, flange_z, rx_final, ry_final, rz_final]
        
        # Move to position with blending (except last point)
        if i < num_points:  # Use blending for all points except the last
            pose_str = ", ".join(f"{v:.6f}" for v in target_pose)
            robot.send_program(f"movel(p[{pose_str}], a={acc}, v={vel}, r={blend_r})")
        else:  # Last point - no blending, wait for completion
            send_movel(robot, target_pose, acc, vel)
            wait_until_pose(robot, target_pose)
        
        angle_x_deg = math.degrees(rx_current)
        angle_y_deg = math.degrees(ry_current)
        print(f"   Point {i+1}/{num_points+1}: X-tilt={angle_x_deg:.1f}°, Y-tilt={angle_y_deg:.1f}°")
    
    print("Conical motion complete!")