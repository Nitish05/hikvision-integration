import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple

import openvr
import pygame


READ_HZ = 90
UI_HZ = 90
PRINT_HZ = 10
WINDOW_SIZE = (960, 720)
MAX_TRAIL_POINTS = 5000


# -----------------------------
# Basic 3x3 matrix helpers
# -----------------------------

Matrix3 = List[List[float]]
Vector3 = Tuple[float, float, float]


def mat3_identity() -> Matrix3:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def mat3_transpose(m: Matrix3) -> Matrix3:
    return [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]


def mat3_mul(a: Matrix3, b: Matrix3) -> Matrix3:
    return [
        [
            a[row][0] * b[0][col] +
            a[row][1] * b[1][col] +
            a[row][2] * b[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def mat3_vec_mul(m: Matrix3, v: Vector3) -> Vector3:
    x, y, z = v
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z,
        m[1][0] * x + m[1][1] * y + m[1][2] * z,
        m[2][0] * x + m[2][1] * y + m[2][2] * z,
    )


def mat3_to_euler_xyz_deg(m: Matrix3) -> Tuple[float, float, float]:
    """
    Convert a 3x3 rotation matrix to XYZ-style Euler angles ONLY for display.

    IMPORTANT:
    The UI object rotation does NOT use these Euler angles anymore.
    Euler angles can still wrap near +/-180 or gimbal lock near +/-90.
    That is normal. The actual 3D rectangle uses the matrix directly.
    """
    r00, r01, r02 = m[0][0], m[0][1], m[0][2]
    r10, r11, r12 = m[1][0], m[1][1], m[1][2]
    r20, r21, r22 = m[2][0], m[2][1], m[2][2]

    if abs(r20) < 0.999999:
        ry = math.asin(-r20)
        rx = math.atan2(r21, r22)
        rz = math.atan2(r10, r00)
    else:
        # Display-only fallback near gimbal lock.
        ry = math.asin(-max(-1.0, min(1.0, r20)))
        rx = 0.0
        rz = math.atan2(-r01, r11)

    return math.degrees(rx), math.degrees(ry), math.degrees(rz)


@dataclass
class ControllerPose:
    device_index: int

    # Position in the active coordinate frame, millimeters.
    x_mm: float
    y_mm: float
    z_mm: float

    # True orientation as a 3x3 rotation matrix.
    # This is what the UI uses. This avoids Euler jumping.
    rot: Matrix3

    # Display-only Euler angles.
    rx_deg: float
    ry_deg: float
    rz_deg: float

    trigger_pressed: bool
    reset_pressed: bool
    updated_at: float


latest_poses = {}
trail_points = {}
pose_lock = threading.Lock()
stop_event = threading.Event()

latest_hmd_pose: Optional[ControllerPose] = None
hmd_reference_pose: Optional[ControllerPose] = None
origin_pose: Optional[ControllerPose] = None
show_test_marker = False
reset_button_was_pressed = False

camera_yaw_deg = 35.0
camera_pitch_deg = 22.0
camera_zoom = 0.26

# OpenVR standing space is X right, Y up, and -Z forward/away from the headset.
# The bridge/robot convention we want to preview is X right, Y away, Z up.
STEAMVR_TO_HEADSET_CARTESIAN: Matrix3 = [
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0],
]


def matrix_to_pose_values(matrix):
    """
    Read SteamVR/OpenVR 3x4 matrix.

    SteamVR returns:
        [ R00 R01 R02 X ]
        [ R10 R11 R12 Y ]
        [ R20 R21 R22 Z ]

    Position is meters. We convert to millimeters.
    Rotation is kept as a true 3x3 matrix.
    """
    x_mm = matrix[0][3] * 1000.0
    y_mm = matrix[1][3] * 1000.0
    z_mm = matrix[2][3] * 1000.0

    rot = [
        [matrix[0][0], matrix[0][1], matrix[0][2]],
        [matrix[1][0], matrix[1][1], matrix[1][2]],
        [matrix[2][0], matrix[2][1], matrix[2][2]],
    ]

    rx_deg, ry_deg, rz_deg = mat3_to_euler_xyz_deg(rot)

    return x_mm, y_mm, z_mm, rot, rx_deg, ry_deg, rz_deg


def pose_from_openvr_matrix(
    device_index: int,
    matrix,
    trigger_pressed: bool,
    reset_pressed: bool,
    updated_at: float,
) -> ControllerPose:
    x_mm, y_mm, z_mm, rot, rx_deg, ry_deg, rz_deg = matrix_to_pose_values(matrix)
    return ControllerPose(
        device_index=device_index,
        x_mm=x_mm,
        y_mm=y_mm,
        z_mm=z_mm,
        rot=rot,
        rx_deg=rx_deg,
        ry_deg=ry_deg,
        rz_deg=rz_deg,
        trigger_pressed=trigger_pressed,
        reset_pressed=reset_pressed,
        updated_at=updated_at,
    )


def transform_pose_to_headset_cartesian(
    pose: ControllerPose,
    hmd_pose: Optional[ControllerPose],
) -> ControllerPose:
    """
    Convert an OpenVR absolute pose into the control frame previewed by the UI.

    Reference:
        origin = headset
        +X = right from headset
        +Y = away/forward from headset
        +Z = up

    OpenVR standing space is close, but uses +Y as up and -Z as forward.
    We first express the controller relative to the headset, then change basis.
    """
    if hmd_pose is None:
        rel_pos_steam = (pose.x_mm, pose.y_mm, pose.z_mm)
        rel_rot_steam = pose.rot
    else:
        hmd_inv = mat3_transpose(hmd_pose.rot)
        delta_pos = (
            pose.x_mm - hmd_pose.x_mm,
            pose.y_mm - hmd_pose.y_mm,
            pose.z_mm - hmd_pose.z_mm,
        )
        rel_pos_steam = mat3_vec_mul(hmd_inv, delta_pos)
        rel_rot_steam = mat3_mul(hmd_inv, pose.rot)

    basis = STEAMVR_TO_HEADSET_CARTESIAN
    basis_inv = mat3_transpose(basis)
    x_mm, y_mm, z_mm = mat3_vec_mul(basis, rel_pos_steam)
    rot = mat3_mul(mat3_mul(basis, rel_rot_steam), basis_inv)
    rx_deg, ry_deg, rz_deg = mat3_to_euler_xyz_deg(rot)

    return ControllerPose(
        device_index=pose.device_index,
        x_mm=x_mm,
        y_mm=y_mm,
        z_mm=z_mm,
        rot=rot,
        rx_deg=rx_deg,
        ry_deg=ry_deg,
        rz_deg=rz_deg,
        trigger_pressed=pose.trigger_pressed,
        reset_pressed=pose.reset_pressed,
        updated_at=pose.updated_at,
    )


def get_controller_buttons(vr, device_index):
    try:
        ok, state = vr.getControllerState(device_index)
    except openvr.OpenVRError:
        return False, False

    if not ok:
        return False, False

    trigger_mask = 1 << openvr.k_EButton_SteamVR_Trigger
    button_pressed = bool(state.ulButtonPressed & trigger_mask)

    # On many OpenVR devices, trigger analog value is axis 1.
    analog_pressed = False
    try:
        analog_pressed = state.rAxis[1].x > 0.5
    except Exception:
        pass

    reset_button_ids = {
        openvr.k_EButton_ApplicationMenu,
        getattr(openvr, "k_EButton_IndexController_B", openvr.k_EButton_ApplicationMenu),
    }
    reset_pressed = any(
        bool(state.ulButtonPressed & (1 << button_id))
        for button_id in reset_button_ids
    )

    return button_pressed or analog_pressed, reset_pressed


def vr_reader():
    global latest_hmd_pose, hmd_reference_pose, origin_pose, reset_button_was_pressed

    openvr.init(openvr.VRApplication_Background)
    vr = openvr.VRSystem()
    period = 1.0 / READ_HZ

    try:
        while not stop_event.is_set():
            start = time.perf_counter()

            poses = vr.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding,
                0,
                openvr.k_unMaxTrackedDeviceCount,
            )

            now = time.time()
            found_raw = {}
            hmd_raw = None
            reset_pressed_any = False

            for i in range(openvr.k_unMaxTrackedDeviceCount):
                device_class = vr.getTrackedDeviceClass(i)
                if device_class not in (
                    openvr.TrackedDeviceClass_HMD,
                    openvr.TrackedDeviceClass_Controller,
                ):
                    continue

                pose = poses[i]
                if not pose.bPoseIsValid:
                    continue

                trigger_pressed = False
                reset_pressed = False
                if device_class == openvr.TrackedDeviceClass_Controller:
                    trigger_pressed, reset_pressed = get_controller_buttons(vr, i)
                    reset_pressed_any = reset_pressed_any or reset_pressed

                raw_pose = pose_from_openvr_matrix(
                    i,
                    pose.mDeviceToAbsoluteTracking,
                    trigger_pressed,
                    reset_pressed,
                    now,
                )

                if device_class == openvr.TrackedDeviceClass_HMD:
                    hmd_raw = raw_pose
                else:
                    found_raw[i] = raw_pose

            with pose_lock:
                if hmd_raw is not None and (
                    hmd_reference_pose is None
                    or (reset_pressed_any and not reset_button_was_pressed)
                ):
                    hmd_reference_pose = hmd_raw
                    origin_pose = None
                    trail_points.clear()
                    print("Reset headset coordinate frame.")

                reset_button_was_pressed = reset_pressed_any
                reference_pose = hmd_reference_pose

            found = {
                i: transform_pose_to_headset_cartesian(raw_pose, reference_pose)
                for i, raw_pose in found_raw.items()
            }
            hmd_cartesian = (
                transform_pose_to_headset_cartesian(hmd_raw, reference_pose)
                if hmd_raw and reference_pose
                else None
            )

            with pose_lock:
                latest_hmd_pose = hmd_cartesian
                latest_poses.clear()
                latest_poses.update(found)

                for pose in found.values():
                    if pose.trigger_pressed:
                        points = trail_points.setdefault(pose.device_index, [])
                        points.append(pose)
                        if len(points) > MAX_TRAIL_POINTS:
                            del points[: len(points) - MAX_TRAIL_POINTS]

            elapsed = time.perf_counter() - start
            time.sleep(max(0.0, period - elapsed))

    finally:
        openvr.shutdown()


def relative_pose(pose: ControllerPose, origin: Optional[ControllerPose]) -> ControllerPose:
    """
    Compute pose relative to reset origin.

    Position:
        relative_position = current_position - origin_position

    Rotation:
        relative_rotation = inverse(origin_rotation) * current_rotation

    For rotation matrices, inverse(rotation) = transpose(rotation).
    This is the important fix.
    Do NOT subtract Euler angles.
    """
    if origin is None:
        return pose

    origin_inv = mat3_transpose(origin.rot)
    rel_rot = mat3_mul(origin_inv, pose.rot)
    rx_deg, ry_deg, rz_deg = mat3_to_euler_xyz_deg(rel_rot)

    return ControllerPose(
        device_index=pose.device_index,
        x_mm=pose.x_mm - origin.x_mm,
        y_mm=pose.y_mm - origin.y_mm,
        z_mm=pose.z_mm - origin.z_mm,
        rot=rel_rot,
        rx_deg=rx_deg,
        ry_deg=ry_deg,
        rz_deg=rz_deg,
        trigger_pressed=pose.trigger_pressed,
        reset_pressed=pose.reset_pressed,
        updated_at=pose.updated_at,
    )


def printer():
    period = 1.0 / PRINT_HZ

    while not stop_event.is_set():
        with pose_lock:
            poses = list(latest_poses.values())
            origin = origin_pose

        poses = [relative_pose(pose, origin) for pose in poses]

        if poses:
            for pose in poses:
                print(
                    f"Controller {pose.device_index}: "
                    f"X={pose.x_mm:8.1f}mm Y={pose.y_mm:8.1f}mm Z={pose.z_mm:8.1f}mm "
                    f"RX={pose.rx_deg:7.2f}deg RY={pose.ry_deg:7.2f}deg RZ={pose.rz_deg:7.2f}deg "
                    f"trigger={'ON' if pose.trigger_pressed else 'off'}"
                )
            print("-" * 96)

        time.sleep(period)


def draw_text(surface, font, text, x, y, color=(235, 238, 242)):
    surface.blit(font.render(text, True, color), (x, y))


def project_3d_point(x_mm, y_mm, z_mm, rect):
    """
    Project real 3D millimeter coordinates to the 2D pygame window.
    This is only camera/view math. It does not change the real pose.

    Control-frame axes are X right, Y away/depth, Z up. The camera math below
    uses right/up/depth internally so the viewport matches that convention.
    """
    yaw = math.radians(camera_yaw_deg)
    pitch = math.radians(camera_pitch_deg)

    x = x_mm
    y = z_mm
    z = y_mm

    x1 = x * math.cos(yaw) - z * math.sin(yaw)
    z1 = x * math.sin(yaw) + z * math.cos(yaw)

    y2 = y * math.cos(pitch) - z1 * math.sin(pitch)
    z2 = y * math.sin(pitch) + z1 * math.cos(pitch)

    depth = 4500.0
    perspective = depth / max(500.0, depth + z2)

    cx = rect.centerx + int(x1 * camera_zoom * perspective)
    cy = rect.centery - int(y2 * camera_zoom * perspective)

    visible = rect.left <= cx <= rect.right and rect.top <= cy <= rect.bottom
    clamped = max(rect.left, min(rect.right, cx)), max(rect.top, min(rect.bottom, cy))

    return clamped, visible, z2


def project_pose_position(pose: ControllerPose, rect):
    return project_3d_point(pose.x_mm, pose.y_mm, pose.z_mm, rect)


def project_plane_point(x_mm, y_mm, z_mm, rect, plane):
    scale_mm_per_px = 8.0

    if plane == "xy":
        sx = rect.centerx + int(x_mm / scale_mm_per_px)
        sy = rect.centery - int(y_mm / scale_mm_per_px)
    elif plane == "xz":
        sx = rect.centerx + int(x_mm / scale_mm_per_px)
        sy = rect.centery - int(z_mm / scale_mm_per_px)
    elif plane == "yz":
        sx = rect.centerx + int(y_mm / scale_mm_per_px)
        sy = rect.centery - int(z_mm / scale_mm_per_px)
    else:
        raise ValueError(f"Unknown plane: {plane}")

    return max(rect.left, min(rect.right, sx)), max(rect.top, min(rect.bottom, sy))


def project_pose_plane(pose: ControllerPose, rect, plane):
    return project_plane_point(pose.x_mm, pose.y_mm, pose.z_mm, rect, plane)


def box_corner(x_mm, y_mm, z_mm):
    return ControllerPose(
        device_index=-1,
        x_mm=x_mm,
        y_mm=y_mm,
        z_mm=z_mm,
        rot=mat3_identity(),
        rx_deg=0.0,
        ry_deg=0.0,
        rz_deg=0.0,
        trigger_pressed=False,
        reset_pressed=False,
        updated_at=0.0,
    )


def draw_3d_box(surface, rect):
    corners = {
        "lnf": box_corner(-1200, -1200, -900),
        "rnf": box_corner(1200, -1200, -900),
        "ltf": box_corner(-1200, -1200, 900),
        "rtf": box_corner(1200, -1200, 900),
        "lnb": box_corner(-1200, 1200, -900),
        "rnb": box_corner(1200, 1200, -900),
        "ltb": box_corner(-1200, 1200, 900),
        "rtb": box_corner(1200, 1200, 900),
    }

    projected = {
        name: project_pose_position(point, rect)[0]
        for name, point in corners.items()
    }

    edges = [
        ("lnf", "rnf"), ("rnf", "rtf"), ("rtf", "ltf"), ("ltf", "lnf"),
        ("lnb", "rnb"), ("rnb", "rtb"), ("rtb", "ltb"), ("ltb", "lnb"),
        ("lnf", "lnb"), ("rnf", "rnb"), ("ltf", "ltb"), ("rtf", "rtb"),
    ]

    for start, end in edges:
        pygame.draw.line(surface, (82, 98, 120), projected[start], projected[end], 2)


def draw_3d_axes(surface, rect):
    origin = box_corner(0, 0, 0)
    axes = [
        ("X", box_corner(900, 0, 0), (255, 111, 145)),
        ("Y", box_corner(0, 900, 0), (130, 219, 108)),
        ("Z", box_corner(0, 0, 900), (91, 192, 235)),
    ]

    origin_point = project_pose_position(origin, rect)[0]

    for label, endpoint, color in axes:
        end_point = project_pose_position(endpoint, rect)[0]
        pygame.draw.line(surface, color, origin_point, end_point, 3)
        pygame.draw.circle(surface, color, end_point, 5)
        draw_text(surface, pygame.font.SysFont("consolas", 14), label, end_point[0] + 6, end_point[1] - 6, color)


def project_local_controller_point(pose: ControllerPose, local_point: Vector3, rect):
    """
    Convert a local controller-model point into screen point.

    1. Rotate local point using pose.rot.
    2. Add controller position.
    3. Project to screen.

    This is the main correction:
    no Euler angles are used to spin the controller rectangle.
    """
    rx, ry, rz = mat3_vec_mul(pose.rot, local_point)

    world_x = pose.x_mm + rx
    world_y = pose.y_mm + ry
    world_z = pose.z_mm + rz

    return project_3d_point(world_x, world_y, world_z, rect)[0]


def draw_controller_box(surface, font, pose: ControllerPose, rect, color):
    width = 180.0
    height = 80.0
    thickness = 45.0

    local_corners = [
        (-width / 2, -height / 2, -thickness / 2),
        (width / 2, -height / 2, -thickness / 2),
        (width / 2, height / 2, -thickness / 2),
        (-width / 2, height / 2, -thickness / 2),
        (-width / 2, -height / 2, thickness / 2),
        (width / 2, -height / 2, thickness / 2),
        (width / 2, height / 2, thickness / 2),
        (-width / 2, height / 2, thickness / 2),
    ]

    world_corners = []
    for c in local_corners:
        rx, ry, rz = mat3_vec_mul(pose.rot, c)
        world_corners.append((pose.x_mm + rx, pose.y_mm + ry, pose.z_mm + rz))

    projected = [project_3d_point(*wc, rect) for wc in world_corners]
    points = [p[0] for p in projected]
    depths = [p[2] for p in projected]

    faces = [
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (2, 3, 7, 6),
        (1, 2, 6, 5),
        (0, 3, 7, 4),
    ]

    # Painter's algorithm: draw back faces first so front faces overlay them.
    faces_sorted = sorted(faces, key=lambda f: -sum(depths[i] for i in f) / len(f))

    fill = (color[0] // 3, color[1] // 3, color[2] // 3)
    for face in faces_sorted:
        face_points = [points[i] for i in face]
        pygame.draw.polygon(surface, fill, face_points)
        pygame.draw.polygon(surface, color, face_points, 2)

    center = project_pose_position(pose, rect)[0]
    axis_len = 170.0
    local_axes = [
        ("+X", (axis_len, 0.0, 0.0), (255, 111, 145)),
        ("+Y", (0.0, axis_len, 0.0), (130, 219, 108)),
        ("+Z", (0.0, 0.0, axis_len), (91, 192, 235)),
    ]
    for label, local_axis, axis_color in local_axes:
        end = project_local_controller_point(pose, local_axis, rect)
        pygame.draw.line(surface, axis_color, center, end, 3)
        pygame.draw.circle(surface, axis_color, end, 4)

    pygame.draw.circle(surface, (255, 255, 255), center, 5)
    draw_text(surface, font, f"C{pose.device_index}", center[0] + 18, center[1] + 12)


def project_local_controller_point_plane(pose: ControllerPose, local_point: Vector3, rect, plane):
    rx, ry, rz = mat3_vec_mul(pose.rot, local_point)
    return project_plane_point(
        pose.x_mm + rx,
        pose.y_mm + ry,
        pose.z_mm + rz,
        rect,
        plane,
    )


def draw_controller_box_plane(surface, font, pose: ControllerPose, rect, color, plane):
    width = 180.0
    height = 80.0
    thickness = 45.0

    corners = [
        (-width / 2, -height / 2, -thickness / 2),
        (width / 2, -height / 2, -thickness / 2),
        (width / 2, height / 2, -thickness / 2),
        (-width / 2, height / 2, -thickness / 2),
        (-width / 2, -height / 2, thickness / 2),
        (width / 2, -height / 2, thickness / 2),
        (width / 2, height / 2, thickness / 2),
        (-width / 2, height / 2, thickness / 2),
    ]
    points = [project_local_controller_point_plane(pose, p, rect, plane) for p in corners]

    faces = [
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (2, 3, 7, 6),
        (1, 2, 6, 5),
        (0, 3, 7, 4),
    ]
    fill = (color[0] // 3, color[1] // 3, color[2] // 3)

    for face in faces:
        face_points = [points[i] for i in face]
        pygame.draw.polygon(surface, fill, face_points)
        pygame.draw.polygon(surface, color, face_points, 2)

    center = project_pose_plane(pose, rect, plane)
    axis_len = 160.0
    local_axes = [
        ("+X", (axis_len, 0.0, 0.0), (255, 111, 145)),
        ("+Y", (0.0, axis_len, 0.0), (130, 219, 108)),
        ("+Z", (0.0, 0.0, axis_len), (91, 192, 235)),
    ]

    for label, local_axis, axis_color in local_axes:
        end = project_local_controller_point_plane(pose, local_axis, rect, plane)
        pygame.draw.line(surface, axis_color, center, end, 3)
        pygame.draw.circle(surface, axis_color, end, 4)

    pygame.draw.circle(surface, (255, 255, 255), center, 5)
    draw_text(surface, font, f"C{pose.device_index}", center[0] + 14, center[1] + 10)


def draw_rotated_rect_2d(surface, center, size, angle_deg, fill_color, outline_color=(255, 255, 255)):
    """
    Still useful for the fake test marker only.
    Real controller drawing does not use this.
    """
    width, height = size
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    corners = [
        (-width / 2, -height / 2),
        (width / 2, -height / 2),
        (width / 2, height / 2),
        (-width / 2, height / 2),
    ]

    points = []

    for x, y in corners:
        points.append((
            center[0] + x * cos_a - y * sin_a,
            center[1] + x * sin_a + y * cos_a,
        ))

    pygame.draw.polygon(surface, fill_color, points)
    pygame.draw.polygon(surface, outline_color, points, 2)


def make_test_pose() -> ControllerPose:
    """
    Fake spinning pose for debugging the UI.
    It builds a rotation matrix directly instead of using Euler drawing.
    """
    angle = math.radians((time.time() * 90.0) % 360.0)
    c = math.cos(angle)
    s = math.sin(angle)

    # Rotate around Z for fake marker.
    rot = [
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ]

    rx, ry, rz = mat3_to_euler_xyz_deg(rot)

    return ControllerPose(
        device_index=99,
        x_mm=0.0,
        y_mm=0.0,
        z_mm=0.0,
        rot=rot,
        rx_deg=rx,
        ry_deg=ry,
        rz_deg=rz,
        trigger_pressed=False,
        reset_pressed=False,
        updated_at=time.time(),
    )


def draw_plane_grid(surface, rect, title, horizontal_label):
    pygame.draw.rect(surface, (82, 98, 120), rect, 3)
    pygame.draw.line(surface, (82, 98, 120), (rect.centerx, rect.top), (rect.centerx, rect.bottom), 1)
    pygame.draw.line(surface, (82, 98, 120), (rect.left, rect.centery), (rect.right, rect.centery), 1)
    pygame.draw.circle(surface, (255, 255, 255), rect.center, 5)
    font = pygame.font.SysFont("consolas", 18)
    small_font = pygame.font.SysFont("consolas", 14)
    draw_text(surface, font, title, rect.left, rect.top - 30)
    draw_text(surface, small_font, horizontal_label, rect.right - 55, rect.centery + 8, (175, 183, 195))
    vertical_label = "Y" if horizontal_label == "X" else "Z"
    draw_text(surface, small_font, vertical_label, rect.centerx + 8, rect.top + 8, (175, 183, 195))


def ui_loop():
    global origin_pose, show_test_marker, camera_yaw_deg, camera_pitch_deg, camera_zoom

    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("VR Controller 6DOF Tracker")

    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)
    small_font = pygame.font.SysFont("consolas", 15)

    view3d_rect = pygame.Rect(30, 50, 900, 550)

    colors = [
        (91, 192, 235),
        (255, 196, 87),
        (130, 219, 108),
        (255, 111, 145),
    ]

    mouse_dragging = False
    mouse_last_pos = None

    while not stop_event.is_set():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stop_event.set()

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_a:
                with pose_lock:
                    if latest_poses:
                        origin_pose = next(iter(latest_poses.values()))
                        print(f"Reset origin to controller {origin_pose.device_index}.")

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_t:
                show_test_marker = not show_test_marker

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                with pose_lock:
                    trail_points.clear()
                print("Cleared drawing.")

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                camera_yaw_deg = 35.0
                camera_pitch_deg = 22.0
                camera_zoom = 0.26

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                camera_yaw_deg -= 5.0

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                camera_yaw_deg += 5.0

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                camera_pitch_deg = min(85.0, camera_pitch_deg + 5.0)

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                camera_pitch_deg = max(-85.0, camera_pitch_deg - 5.0)

            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                camera_zoom = min(0.8, camera_zoom * 1.15)

            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_MINUS, pygame.K_UNDERSCORE):
                camera_zoom = max(0.05, camera_zoom / 1.15)

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if view3d_rect.collidepoint(event.pos):
                    mouse_dragging = True
                    mouse_last_pos = event.pos

            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                mouse_dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if mouse_dragging and mouse_last_pos:
                    dx = event.pos[0] - mouse_last_pos[0]
                    dy = event.pos[1] - mouse_last_pos[1]
                    camera_yaw_deg += dx * 0.4
                    camera_pitch_deg = max(-85.0, min(85.0, camera_pitch_deg - dy * 0.4))
                    mouse_last_pos = event.pos

            elif event.type == pygame.MOUSEWHEEL:
                camera_zoom = min(0.8, camera_zoom * (1.1 if event.y > 0 else 1.0 / 1.1))

        with pose_lock:
            poses = list(latest_poses.values())
            origin = origin_pose
            trails = {
                device_index: [relative_pose(point, origin) for point in points]
                for device_index, points in trail_points.items()
            }

        relative_poses = [relative_pose(pose, origin) for pose in poses]

        if show_test_marker:
            relative_poses.append(make_test_pose())

        screen.fill((18, 22, 28))

        draw_text(screen, font, "VR Controller 6DOF Tracker", 30, 15)
        draw_text(screen, small_font,
                  f"yaw {camera_yaw_deg:.0f}  pitch {camera_pitch_deg:.0f}  zoom {camera_zoom:.2f}",
                  650, 19, (175, 183, 195))

        # 3D view
        pygame.draw.rect(screen, (52, 68, 90), view3d_rect, 2)
        draw_3d_box(screen, view3d_rect)
        draw_3d_axes(screen, view3d_rect)

        origin_label = ""
        if origin is not None:
            origin_label = f"   origin: C{origin.device_index}"

        draw_text(screen, small_font,
                  f"Orbit: drag/arrows  |  Zoom: wheel/+/-  |  R: view  |  B/Y: headset frame  |  A: origin{origin_label}  |  C: clear  |  T: test",
                  30, 610, (175, 183, 195))

        if not relative_poses:
            draw_text(screen, font, "No valid controller pose. Start SteamVR and wake controllers.", 30, 640, (255, 196, 87))
        else:
            # Draw trails in 3D.
            for device_index, points in trails.items():
                if len(points) < 2:
                    continue
                color = colors[device_index % len(colors)]
                trail_pts = [project_pose_position(p, view3d_rect)[0] for p in points]
                pygame.draw.lines(screen, color, False, trail_pts, 2)

            # Draw controller boxes in 3D.
            for idx, pose in enumerate(relative_poses):
                color = colors[idx % len(colors)]
                draw_controller_box(screen, small_font, pose, view3d_rect, color)

            # Numerical readout.
            y = 635
            for pose in relative_poses[:4]:
                draw_text(
                    screen, small_font,
                    (
                        f"C{pose.device_index}: "
                        f"X {pose.x_mm:+8.1f}  Y {pose.y_mm:+8.1f}  Z {pose.z_mm:+8.1f} mm   "
                        f"RX {pose.rx_deg:+7.2f}  RY {pose.ry_deg:+7.2f}  RZ {pose.rz_deg:+7.2f} deg   "
                        f"trigger {'ON ' if pose.trigger_pressed else 'off'}"
                    ),
                    30, y, (235, 238, 242),
                )
                y += 22

        pygame.display.flip()
        clock.tick(UI_HZ)

    pygame.quit()


def main():
    print("Reading controller positions in a background thread.")
    print("X/Y/Z are headset-relative millimeters: +X right, +Y away, +Z up.")
    print("The first valid headset pose locks the coordinate frame. Press B/Y to reset it.")
    print("RX/RY/RZ are display-only Euler angles.")
    print("The 3D UI rotation uses the headset-relative converted rotation matrix directly.")
    print("Close the UI or press Ctrl+C to stop.")

    reader_thread = threading.Thread(target=vr_reader, daemon=True)
    printer_thread = threading.Thread(target=printer, daemon=True)

    reader_thread.start()
    printer_thread.start()

    try:
        ui_loop()

    except KeyboardInterrupt:
        stop_event.set()

    finally:
        stop_event.set()
        reader_thread.join(timeout=2)
        printer_thread.join(timeout=2)
        print("Stopped.")


if __name__ == "__main__":
    main()
