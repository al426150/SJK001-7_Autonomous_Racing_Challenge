import math
import cv2
import WebGUI
import HAL
import Frequency

# ==============================================================================
# 1. PARAMETERS & REFERENCE GPS COORDINATES
# ==============================================================================
BOAT_LAT = 40.0 + 16.0 / 60.0 + 48.2 / 3600.0
BOAT_LON = -(3.0 + 49.0 / 60.0 + 3.5 / 3600.0)

SURVIVORS_LAT = 40.0 + 16.0 / 60.0 + 47.23 / 3600.0
SURVIVORS_LON = -(3.0 + 49.0 / 60.0 + 1.78 / 3600.0)

M_PER_LAT = 111132.9
M_PER_LON = 111319.5 * math.cos(math.radians(BOAT_LAT))

ALTITUDE = 4.0                 # Search altitude above deck (m)
SPIRAL_SPACING = 2.8           # Radial spacing between spiral loops (m)
THETA_STEP = math.radians(15)  # Angular increment (15 deg)
EXPECTED_SURVIVORS = 6         # Early-stop target survivor count
MAX_RADIUS = 20.0              # Maximum spiral search radius (m)
DUPLICATE_RADIUS = 3.2         # Minimum distance (m) between projected survivor positions
CAMERA_FOV_SCALE = 1.65        # Calibrated ventral camera ground-width / altitude

# Velocity & landing control parameters (tuned for 30 Hz loop)
OUTBOUND_SPEED = 5.5           # Fast cruise speed (m/s) toward survivors' zone
RETURN_SPEED = 2.8             # Controlled return speed (m/s) to prevent boat overshoot
SPIRAL_SPEED = 2.8             # Tangential speed (m/s) during SPIRAL search
TRANSIT_TOLERANCE = 0.7        # Waypoint tolerance (m) at search center
ALIGN_HANDOFF_DIST = 3.0       # Distance (m) from boat to switch to position hold
LAND_TOLERANCE = 0.55          # Horizontal acceptance radius (m) over the safety boat
SPIRAL_TOLERANCE = 1.15        # Lookahead tolerance (m) to switch waypoints without braking
STABILIZE_TICKS = 25           # ~0.8 s stationary confirmation at 30 Hz before landing

face_cascade = cv2.CascadeClassifier(
    "/resources/exercises/rescue_people/haarcascade_frontalface_default.xml"
)


# ==============================================================================
# 2. VISION, GEOMETRY & VELOCITY CONTROLLER HELPERS
# ==============================================================================
def rotate_img(img, angle):
    if angle == 0:
        return img
    h, w = img.shape[:2]
    m = cv2.getRotationMatrix2D((w // 2, h // 2), angle, 1.0)
    return cv2.warpAffine(img, m, (w, h))


def detect_faces(bgr_img):
    """
    Detects faces across 8 orientations within 84% of the frame and returns
    their unrotated pixel offsets (dx, dy) from the image center.
    """
    img_h, img_w = bgr_img.shape[:2]
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

    for angle in [0, 90, 180, 270, 45, 135, 225, 315]:
        rot_gray = rotate_img(gray, angle)
        faces = face_cascade.detectMultiScale(
            rot_gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(22, 22)
        )
        valid_faces = [
            (x, y, w, h) for (x, y, w, h) in faces
            if (0.08 * img_w <= x + w / 2.0 <= 0.92 * img_w)
            and (0.08 * img_h <= y + h / 2.0 <= 0.92 * img_h)
        ]
        if len(valid_faces) > 0:
            annotated = rotate_img(bgr_img.copy(), angle)
            offsets = []
            rad = math.radians(angle)
            cos_a, sin_a = math.cos(rad), math.sin(rad)

            for (x, y, w, h) in valid_faces:
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
                dx_rot = (x + w / 2.0) - (img_w / 2.0)
                dy_rot = (y + h / 2.0) - (img_h / 2.0)
                dx_orig = cos_a * dx_rot - sin_a * dy_rot
                dy_orig = sin_a * dx_rot + cos_a * dy_rot
                offsets.append((dx_orig, dy_orig))

            return offsets, annotated, cv2.cvtColor(rot_gray, cv2.COLOR_GRAY2BGR)

    return [], bgr_img.copy(), cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def dist_2d(cx, cy, tx, ty):
    return math.hypot(tx - cx, ty - cy)


def compute_cmd_vel(cx, cy, cz, yaw, tx, ty, tz, max_speed, brake_gain=None, min_speed=0.20):
    """
    Computes body-frame velocity commands (vx, vy, vz, az) to reach (tx, ty, tz)
    while holding yaw = 0 rad.
    """
    dx = tx - cx
    dy = ty - cy
    dist = math.hypot(dx, dy)

    if dist > 1e-4:
        if brake_gain is not None:
            speed = min(max_speed, max(min_speed, brake_gain * dist))
        else:
            speed = max_speed

        vx_world = (dx / dist) * speed
        vy_world = (dy / dist) * speed
    else:
        vx_world, vy_world = 0.0, 0.0

    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    vx_body = cos_y * vx_world + sin_y * vy_world
    vy_body = -sin_y * vx_world + cos_y * vy_world

    vz = max(-1.5, min(1.5, 1.5 * (tz - cz)))
    yaw_err = math.atan2(math.sin(-yaw), math.cos(-yaw))
    az = max(-1.0, min(1.0, 1.5 * yaw_err))

    return vx_body, vy_body, vz, az


# ==============================================================================
# 3. INITIALIZATION & CONTROL LOOP
# ==============================================================================
init_pos = HAL.get_position()
start_x, start_y, start_z = float(init_pos[0]), float(init_pos[1]), float(init_pos[2])

center_x = start_x + (SURVIVORS_LON - BOAT_LON) * M_PER_LON
center_y = start_y + (SURVIVORS_LAT - BOAT_LAT) * M_PER_LAT
target_alt = start_z + ALTITUDE

print(f"[INIT] Taking off to {ALTITUDE} m above deck. Search center: ({center_x:.2f}, {center_y:.2f}) m")
HAL.takeoff(target_alt)

state = "TAKEOFF"
theta = math.radians(180)
r = 0.0
target_x, target_y = center_x, center_y
rescued = []
ticks = 0
stabilize_ticks = 0

while True:
    Frequency.tick()
    ticks += 1

    pos = HAL.get_position()
    cx, cy, cz = float(pos[0]), float(pos[1]), float(pos[2])
    yaw = float(HAL.get_yaw())

    img = HAL.get_ventral_image()
    if img is not None:
        if state == "SPIRAL":
            face_offsets, right_img, left_img = detect_faces(img)
            if face_offsets:
                img_w = img.shape[1]
                meters_per_px = (max(cz, 1.0) * CAMERA_FOV_SCALE) / img_w

                for dx_px, dy_px in face_offsets:
                    face_x = cx - dy_px * meters_per_px
                    face_y = cy - dx_px * meters_per_px

                    if all(math.hypot(face_x - p["x"], face_y - p["y"]) >= DUPLICATE_RADIUS for p in rescued):
                        lat = BOAT_LAT + (face_y - start_y) / M_PER_LAT
                        lon = BOAT_LON + (face_x - start_x) / M_PER_LON
                        rescued.append({"id": len(rescued) + 1, "x": face_x, "y": face_y, "lat": lat, "lon": lon})
                        print(
                            f"[DETECTION {len(rescued)}/{EXPECTED_SURVIVORS}] "
                            f"GPS: ({lat:.6f}, {lon:.6f}) | Survivor ENU: ({face_x:.2f}, {face_y:.2f}) m"
                        )
        else:
            right_img = img.copy()
            left_img = cv2.cvtColor(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)

        cv2.putText(
            right_img, f"{state} (VEL) | Found: {len(rescued)}/{EXPECTED_SURVIVORS}",
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2
        )
        WebGUI.showImage(right_img)
        WebGUI.showLeftImage(left_img)

    if ticks % 30 == 0 and state != "FINISHED":
        print(
            f"[{state}] Pos: ({cx:.2f}, {cy:.2f}, {cz:.2f}) | "
            f"r={r:.1f} m | Found: {len(rescued)}/{EXPECTED_SURVIVORS}"
        )

    if state == "TAKEOFF" and cz >= target_alt - 0.5:
        print("[NAV] Target altitude reached. Flying fast toward survivors' zone...")
        state = "GO_TO_ZONE"

    elif state == "GO_TO_ZONE":
        vx, vy, vz, az = compute_cmd_vel(
            cx, cy, cz, yaw, center_x, center_y, target_alt,
            max_speed=OUTBOUND_SPEED, brake_gain=0.55, min_speed=0.25
        )
        HAL.set_cmd_vel(vx, vy, vz, az)

        if dist_2d(cx, cy, center_x, center_y) < TRANSIT_TOLERANCE:
            print("[SEARCH] Search center reached. Starting high-speed Archimedean Spiral...")
            r = SPIRAL_SPACING * (theta / (2.0 * math.pi))
            target_x, target_y = center_x + r * math.cos(theta), center_y + r * math.sin(theta)
            state = "SPIRAL"

    elif state == "SPIRAL":
        vx, vy, vz, az = compute_cmd_vel(
            cx, cy, cz, yaw, target_x, target_y, target_alt,
            max_speed=SPIRAL_SPEED, brake_gain=None
        )
        HAL.set_cmd_vel(vx, vy, vz, az)

        if len(rescued) >= EXPECTED_SURVIVORS or r > MAX_RADIUS:
            reason = f"{EXPECTED_SURVIVORS} survivors located" if len(rescued) >= EXPECTED_SURVIVORS else f"max radius ({MAX_RADIUS} m)"
            print(f"[RETURN] Search completed ({reason}). Returning to safety boat...")
            state = "RETURN"
        elif dist_2d(cx, cy, target_x, target_y) < SPIRAL_TOLERANCE:
            theta += THETA_STEP
            r = SPIRAL_SPACING * (theta / (2.0 * math.pi))
            target_x = center_x + r * math.cos(theta)
            target_y = center_y + r * math.sin(theta)

    elif state == "RETURN":
        vx, vy, vz, az = compute_cmd_vel(
            cx, cy, cz, yaw, start_x, start_y, target_alt,
            max_speed=RETURN_SPEED, brake_gain=0.38, min_speed=0.18
        )
        HAL.set_cmd_vel(vx, vy, vz, az)

        if dist_2d(cx, cy, start_x, start_y) < ALIGN_HANDOFF_DIST:
            print("[ALIGN] Near safety boat. Holding position for touchdown...")
            stabilize_ticks = 0
            state = "ALIGN_BOAT"

    elif state == "ALIGN_BOAT":
        HAL.set_cmd_pos(start_x, start_y, target_alt, 0.0)

        if dist_2d(cx, cy, start_x, start_y) < LAND_TOLERANCE:
            stabilize_ticks += 1
            if stabilize_ticks >= STABILIZE_TICKS:
                print("[LAND] Centered over safety boat. Initiating landing...")
                HAL.land()
                state = "LANDING"
        else:
            stabilize_ticks = 0

    elif state == "LANDING" and cz <= start_z + 0.25:
        state = "FINISHED"
        print("\n=== FINAL RESCUE REPORT ===")
        if not rescued:
            print("No survivors were detected during the search.")
        else:
            for p in rescued:
                print(
                    f"Survivor #{p['id']}: GPS = ({p['lat']:.6f}, {p['lon']:.6f}) | "
                    f"Local ENU = ({p['x']:.2f}, {p['y']:.2f}) m"
                )