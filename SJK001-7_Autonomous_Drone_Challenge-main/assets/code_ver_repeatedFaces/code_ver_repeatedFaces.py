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

ALTITUDE = 4.0                 # Search altitude (m)
SPIRAL_SPACING = 3.5           # b' = 3.5 m radial spacing between spiral loops
THETA_STEP = math.radians(20)  # Angular increment (20 deg)
TOLERANCE = 1.0                # Waypoint acceptance radius (m)
EXPECTED_SURVIVORS = 6         # Early-stop target survivor count
MAX_RADIUS = 25.0              # Maximum spiral search radius (m)
DUPLICATE_RADIUS = 2.5         # Minimum distance (m) between distinct victims

face_cascade = cv2.CascadeClassifier(
    "/resources/exercises/rescue_people/haarcascade_frontalface_default.xml"
)


# ==============================================================================
# 2. VISION & GEOMETRY HELPER FUNCTIONS
# ==============================================================================
def rotate_img(img, angle):
    if angle == 0:
        return img
    h, w = img.shape[:2]
    m = cv2.getRotationMatrix2D((w // 2, h // 2), angle, 1.0)
    return cv2.warpAffine(img, m, (w, h))


def detect_faces(bgr_img):
    """Detects faces across 8 orientations without histogram equalization."""
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

    for angle in [0, 90, 180, 270, 45, 135, 225, 315]:
        rot_gray = rotate_img(gray, angle)
        faces = face_cascade.detectMultiScale(
            rot_gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(22, 22)
        )
        if len(faces) > 0:
            annotated = rotate_img(bgr_img.copy(), angle)
            for (x, y, w, h) in faces:
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            return True, annotated, cv2.cvtColor(rot_gray, cv2.COLOR_GRAY2BGR)

    return False, bgr_img.copy(), cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def dist_3d(pos, tx, ty, tz):
    return math.sqrt((tx - pos[0])**2 + (ty - pos[1])**2 + (tz - pos[2])**2)


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
theta = math.radians(20)
r = 0.0
target_x, target_y = center_x, center_y
rescued = []
ticks = 0

while True:
    Frequency.tick()
    ticks += 1

    pos = HAL.get_position()
    cx, cy, cz = float(pos[0]), float(pos[1]), float(pos[2])

    img = HAL.get_ventral_image()
    if img is not None:
        if state == "SPIRAL":
            found, right_img, left_img = detect_faces(img)
            if found and all(math.hypot(cx - p["x"], cy - p["y"]) >= DUPLICATE_RADIUS for p in rescued):
                lat = BOAT_LAT + (cy - start_y) / M_PER_LAT
                lon = BOAT_LON + (cx - start_x) / M_PER_LON
                rescued.append({"id": len(rescued) + 1, "x": cx, "y": cy, "lat": lat, "lon": lon})
                print(
                    f"[DETECTION {len(rescued)}/{EXPECTED_SURVIVORS}] "
                    f"GPS: ({lat:.6f}, {lon:.6f}) | Local ENU: ({cx:.2f}, {cy:.2f}) m"
                )
        else:
            right_img = img.copy()
            left_img = cv2.cvtColor(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)

        cv2.putText(
            right_img, f"{state} | Found: {len(rescued)}/{EXPECTED_SURVIVORS}",
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2
        )
        WebGUI.showImage(right_img)
        WebGUI.showLeftImage(left_img)

    if ticks % 50 == 0 and state != "FINISHED":
        print(
            f"[{state}] Pos: ({cx:.2f}, {cy:.2f}, {cz:.2f}) | "
            f"r={r:.1f} m | Found: {len(rescued)}/{EXPECTED_SURVIVORS}"
        )

    if state == "TAKEOFF" and cz >= target_alt - 0.5:
        print("[NAV] Target altitude reached. Flying toward survivors' zone...")
        state = "GO_TO_ZONE"

    elif state == "GO_TO_ZONE":
        HAL.set_cmd_pos(center_x, center_y, target_alt, 0.0)
        if dist_3d(pos, center_x, center_y, target_alt) < TOLERANCE:
            print("[SEARCH] Search center reached. Starting Archimedean Spiral...")
            r = SPIRAL_SPACING * (theta / (2.0 * math.pi))
            target_x, target_y = center_x + r * math.cos(theta), center_y + r * math.sin(theta)
            state = "SPIRAL"

    elif state == "SPIRAL":
        HAL.set_cmd_pos(target_x, target_y, target_alt, 0.0)

        if len(rescued) >= EXPECTED_SURVIVORS or r > MAX_RADIUS:
            reason = f"{EXPECTED_SURVIVORS} survivors located" if len(rescued) >= EXPECTED_SURVIVORS else f"max radius ({MAX_RADIUS} m)"
            print(f"[RETURN] Search completed ({reason}). Returning to safety boat...")
            target_x, target_y = start_x, start_y
            state = "RETURN"
        elif dist_3d(pos, target_x, target_y, target_alt) < TOLERANCE:
            theta += THETA_STEP
            r = SPIRAL_SPACING * (theta / (2.0 * math.pi))
            target_x = center_x + r * math.cos(theta)
            target_y = center_y + r * math.sin(theta)

    elif state == "RETURN":
        HAL.set_cmd_pos(start_x, start_y, target_alt, 0.0)
        if dist_3d(pos, start_x, start_y, target_alt) < TOLERANCE:
            print("[LAND] Over safety boat. Initiating landing...")
            HAL.land()
            state = "LANDING"

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