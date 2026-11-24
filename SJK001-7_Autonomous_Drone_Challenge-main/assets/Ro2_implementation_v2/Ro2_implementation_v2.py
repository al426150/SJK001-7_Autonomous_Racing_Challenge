# pylint: disable=no-member,import-error,unused-import,broad-except
import math
import cv2
import WebGUI  # Initializes rclpy internally and enables /webgui/image_debug_* topics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from as2_msgs.msg import ControlMode, PlatformInfo, PlatformStateMachineEvent
from as2_msgs.srv import SetControlMode, SetPlatformStateMachineEvent
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


# ==============================================================================
# 1. REFERENCE GPS COORDINATES & MISSION PARAMETERS
# ==============================================================================
BOAT_LAT = 40.0 + 16.0 / 60.0 + 48.2 / 3600.0
BOAT_LON = -(3.0 + 49.0 / 60.0 + 3.5 / 3600.0)

SURVIVORS_LAT = 40.0 + 16.0 / 60.0 + 47.23 / 3600.0
SURVIVORS_LON = -(3.0 + 49.0 / 60.0 + 1.78 / 3600.0)

M_PER_LAT = 111132.9
M_PER_LON = 111319.5 * math.cos(math.radians(BOAT_LAT))

CONTROL_HZ = 30.0              # ROS 2 control loop frequency
ALTITUDE = 4.0                 # Search altitude above boat deck (m)
SPIRAL_SPACING = 2.8           # Radial spacing between spiral loops (m)
THETA_STEP = math.radians(15)  # Angular increment per waypoint (15 deg)
EXPECTED_SURVIVORS = 6         # Target survivor count for early stop
MAX_RADIUS = 20.0              # Maximum spiral search radius (m)
DUPLICATE_RADIUS = 3.2         # Minimum distance (m) between distinct survivors
CAMERA_FOV_SCALE = 1.65        # Calibrated ventral camera ground-width / altitude ratio

# Hybrid velocity & precision landing parameters
OUTBOUND_SPEED = 5.5           # Fast cruise speed (m/s) toward survivors' zone
RETURN_SPEED = 2.8             # Controlled return speed (m/s) to prevent boat overshoot
SPIRAL_SPEED = 2.8             # Tangential speed (m/s) during SPIRAL search
TRANSIT_TOLERANCE = 0.7        # Waypoint tolerance (m) at search center
ALIGN_HANDOFF_DIST = 3.0       # Distance (m) from boat to switch to position hold
LAND_TOLERANCE = 0.65          # Horizontal acceptance radius (m) over the safety boat
SPIRAL_TOLERANCE = 1.15        # Lookahead tolerance (m) to switch waypoints without braking
STABILIZE_TICKS = 20           # ~0.67 s stationary confirmation before landing

CASCADE_PATH = "/resources/exercises/rescue_people/haarcascade_frontalface_default.xml"


# ==============================================================================
# 2. GEOMETRY & COMPUTER VISION HELPER FUNCTIONS
# ==============================================================================
def quaternion_to_yaw(qx, qy, qz, qw):
    """Extracts yaw angle (rad) in ENU frame from a unit quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_img(img, angle):
    if angle == 0:
        return img
    h, w = img.shape[:2]
    m = cv2.getRotationMatrix2D((w // 2, h // 2), angle, 1.0)
    return cv2.warpAffine(img, m, (w, h))


def detect_faces(bgr_img, face_cascade):
    """
    Detects faces across 8 orientations within the central 84% of the frame and
    returns their unrotated pixel offsets (dx, dy) from the image center.
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


def compute_enu_twist(cx, cy, cz, yaw, tx, ty, tz, max_speed, brake_gain=None, min_speed=0.20):
    """
    Computes velocity commands in ENU frame (East +X, North +Y, Up +Z, YawRate)
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
        vx_enu = (dx / dist) * speed
        vy_enu = (dy / dist) * speed
    else:
        vx_enu, vy_enu = 0.0, 0.0

    vz = max(-1.5, min(1.5, 1.5 * (tz - cz)))
    yaw_err = math.atan2(math.sin(-yaw), math.cos(-yaw))
    az = max(-1.0, min(1.0, 1.5 * yaw_err))

    return vx_enu, vy_enu, vz, az


# ==============================================================================
# 3. NATIVE ROS 2 / AEROSTACK2 NODE (WITHOUT HAL)
# ==============================================================================
class RescuePeopleDirectNode(Node):
    def __init__(self):
        super().__init__("rescue_people_direct_node")

        self.face_cascade = cv2.CascadeClassifier(CASCADE_PATH)
        self.bridge = CvBridge()

        # TF2 listener for continuous local ENU pose lookup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pair = None

        # Platform & localization state
        self.platform_info = PlatformInfo()
        self.info_received = False
        self.pose_topic_active = False
        self.pose_received = False
        self.frame_id = "earth"

        self.cx = 0.0
        self.cy = 0.0
        self.cz = 0.0
        self.yaw = 0.0
        self.vz_meas = 0.0
        self.latest_img = None

        # Mission state machine variables
        self.state = "WAIT_CONNECTION"
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_z = 0.0
        self.center_x = 0.0
        self.center_y = 0.0
        self.target_alt = ALTITUDE

        self.theta = math.radians(180)
        self.r = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.rescued = []

        self.ticks = 0
        self.stabilize_ticks = 0
        self.last_srv_tick = -30
        self.last_mode_tick = -30

        # Native Aerostack2 / Unibotics subscribers
        self.sub_info = self.create_subscription(
            PlatformInfo, "/drone/platform/info", self.cb_platform_info, qos_profile_sensor_data
        )
        self.sub_twist = self.create_subscription(
            TwistStamped, "/drone/self_localization/twist", self.cb_twist, qos_profile_sensor_data
        )
        self.sub_pose = self.create_subscription(
            PoseStamped, "/drone/self_localization/pose", self.cb_pose, qos_profile_sensor_data
        )
        self.sub_ventral = self.create_subscription(
            Image, "/drone/ventral_cam/image_raw", self.cb_ventral_image, qos_profile_sensor_data
        )

        # Motion reference & WebGUI debug image publishers
        self.pub_pose = self.create_publisher(PoseStamped, "/drone/motion_reference/pose", 10)
        self.pub_twist = self.create_publisher(TwistStamped, "/drone/motion_reference/twist", 10)
        self.pub_gui_right = self.create_publisher(Image, "/webgui/image_debug_right", 10)
        self.pub_gui_left = self.create_publisher(Image, "/webgui/image_debug_left", 10)

        # Aerostack2 service clients
        self.cli_arm = self.create_client(SetBool, "/drone/set_arming_state")
        self.cli_offboard = self.create_client(SetBool, "/drone/set_offboard_mode")
        self.cli_sm_event = self.create_client(
            SetPlatformStateMachineEvent, "/drone/platform/state_machine_event"
        )
        self.cli_ctrl_mode = self.create_client(
            SetControlMode, "/drone/controller/set_control_mode"
        )

        # Control loop timer
        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.control_loop)
        self.log("[INIT] Direct ROS 2 node started. Waiting for /drone telemetry...")

    def log(self, msg):
        """Prints status messages to both Unibotics Console (stdout) and /rosout."""
        print(msg, flush=True)
        self.get_logger().info(msg)

    # --------------------------------------------------------------------------
    # Subscriber Callbacks & Continuous TF2 Pose Lookup
    # --------------------------------------------------------------------------
    def cb_platform_info(self, msg: PlatformInfo):
        self.platform_info = msg
        self.info_received = True

    def cb_twist(self, msg: TwistStamped):
        self.vz_meas = float(msg.twist.linear.z)

    def cb_pose(self, msg: PoseStamped):
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id
        self.cx = float(msg.pose.position.x)
        self.cy = float(msg.pose.position.y)
        self.cz = float(msg.pose.position.z)
        q = msg.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.pose_topic_active = True
        self.pose_received = True

    def cb_ventral_image(self, msg: Image):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Error converting ventral image: {exc}")

    def read_tf_pose(self):
        candidates = (
            [self.tf_pair]
            if self.tf_pair is not None
            else [
                ("earth", "drone/base_link"),
                ("earth", "drone"),
                ("map", "drone/base_link"),
                ("drone/odom", "drone/base_link"),
            ]
        )
        for parent, child in candidates:
            try:
                tf_msg = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                self.tf_pair = (parent, child)
                self.frame_id = parent
                self.cx = float(tf_msg.transform.translation.x)
                self.cy = float(tf_msg.transform.translation.y)
                self.cz = float(tf_msg.transform.translation.z)
                q = tf_msg.transform.rotation
                self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
                self.pose_received = True
                return True
            except Exception:
                continue
        return False

    # --------------------------------------------------------------------------
    # Aerostack2 Services & Motion Reference Publishers
    # --------------------------------------------------------------------------
    def call_set_bool(self, client, value):
        if client.service_is_ready():
            req = SetBool.Request()
            req.data = bool(value)
            client.call_async(req)

    def call_sm_event(self, event_id):
        if self.cli_sm_event.service_is_ready():
            req = SetPlatformStateMachineEvent.Request()
            req.event.event = int(event_id)
            self.cli_sm_event.call_async(req)

    def set_controller_mode(self, mode_id, yaw_mode_id, ref_frame_id):
        cur = self.platform_info.current_control_mode
        if (
            cur.control_mode == mode_id
            and cur.yaw_mode == yaw_mode_id
            and cur.reference_frame == ref_frame_id
        ):
            return
        if self.ticks - self.last_mode_tick < 5:
            return
        if self.cli_ctrl_mode.service_is_ready():
            self.last_mode_tick = self.ticks
            req = SetControlMode.Request()
            req.control_mode.control_mode = int(mode_id)
            req.control_mode.yaw_mode = int(yaw_mode_id)
            req.control_mode.reference_frame = int(ref_frame_id)
            self.cli_ctrl_mode.call_async(req)

    def publish_pose(self, x, y, z, yaw_rad=0.0):
        self.set_controller_mode(
            getattr(ControlMode, "POSITION", 2),
            getattr(ControlMode, "YAW_ANGLE", 1),
            getattr(ControlMode, "LOCAL_ENU_FRAME", 1),
        )
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self.pub_pose.publish(msg)

    def publish_twist(self, vx_enu, vy_enu, vz, az):
        self.set_controller_mode(
            getattr(ControlMode, "SPEED", 3),
            getattr(ControlMode, "YAW_SPEED", 2),
            getattr(ControlMode, "LOCAL_ENU_FRAME", 1),
        )
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = float(vx_enu)
        msg.twist.linear.y = float(vy_enu)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(az)
        self.pub_twist.publish(msg)

    # --------------------------------------------------------------------------
    # Control & Vision Loop
    # --------------------------------------------------------------------------
    def control_loop(self):
        self.ticks += 1

        if not self.pose_topic_active:
            self.read_tf_pose()

        # 1. Process ventral camera feed & publish debug images to WebGUI
        if self.latest_img is not None:
            img = self.latest_img
            if self.state == "SPIRAL":
                face_offsets, right_img, left_img = detect_faces(img, self.face_cascade)
                if face_offsets:
                    img_w = img.shape[1]
                    meters_per_px = (max(self.cz, 1.0) * CAMERA_FOV_SCALE) / img_w

                    for dx_px, dy_px in face_offsets:
                        face_x = self.cx - dy_px * meters_per_px
                        face_y = self.cy - dx_px * meters_per_px

                        if all(math.hypot(face_x - p["x"], face_y - p["y"]) >= DUPLICATE_RADIUS for p in self.rescued):
                            lat = BOAT_LAT + (face_y - self.start_y) / M_PER_LAT
                            lon = BOAT_LON + (face_x - self.start_x) / M_PER_LON
                            self.rescued.append({
                                "id": len(self.rescued) + 1,
                                "x": face_x,
                                "y": face_y,
                                "lat": lat,
                                "lon": lon
                            })
                            self.log(
                                f"[DETECTION {len(self.rescued)}/{EXPECTED_SURVIVORS}] "
                                f"GPS: ({lat:.6f}, {lon:.6f}) | Local ENU: ({face_x:.2f}, {face_y:.2f}) m"
                            )
            else:
                right_img = img.copy()
                left_img = cv2.cvtColor(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)

            cv2.putText(
                right_img, f"{self.state} (ROS2) | Found: {len(self.rescued)}/{EXPECTED_SURVIVORS}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2
            )
            self.pub_gui_right.publish(self.bridge.cv2_to_imgmsg(right_img, encoding="bgr8"))
            self.pub_gui_left.publish(self.bridge.cv2_to_imgmsg(left_img, encoding="bgr8"))

        # Periodic status log every 1.5 s
        if self.ticks % 45 == 0 and self.state not in ("WAIT_CONNECTION", "FINISHED"):
            self.log(
                f"[{self.state}] Pos: ({self.cx:.2f}, {self.cy:.2f}, {self.cz:.2f}) | "
                f"r={self.r:.1f} m | Found: {len(self.rescued)}/{EXPECTED_SURVIVORS}"
            )

        # Mission finite state machine
        if self.state == "WAIT_CONNECTION":
            if self.info_received and self.pose_received:
                self.start_x, self.start_y, self.start_z = self.cx, self.cy, self.cz
                self.center_x = self.start_x + (SURVIVORS_LON - BOAT_LON) * M_PER_LON
                self.center_y = self.start_y + (SURVIVORS_LAT - BOAT_LAT) * M_PER_LAT
                self.target_alt = self.start_z + ALTITUDE
                self.log(
                    f"[INIT] Connected to Aerostack2 ({self.tf_pair}). Taking off to {ALTITUDE} m above deck "
                    f"({self.start_x:.2f}, {self.start_y:.2f}, {self.start_z:.2f}) m. "
                    f"Search center: ({self.center_x:.2f}, {self.center_y:.2f}) m"
                )
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            p_state = self.platform_info.status.state
            self.publish_pose(self.start_x, self.start_y, self.target_alt, 0.0)

            if self.ticks - self.last_srv_tick >= 10:
                self.last_srv_tick = self.ticks
                if not self.platform_info.armed:
                    self.call_set_bool(self.cli_arm, True)
                if not self.platform_info.offboard:
                    self.call_set_bool(self.cli_offboard, True)

                if p_state == 0:    # DISARMED -> ARM
                    self.call_sm_event(PlatformStateMachineEvent.ARM)
                elif p_state == 1:  # LANDED -> TAKE_OFF
                    self.call_sm_event(PlatformStateMachineEvent.TAKE_OFF)
                elif p_state == 2:  # TAKING_OFF -> TOOK_OFF (transitions to FLYING = 3)
                    self.call_sm_event(PlatformStateMachineEvent.TOOK_OFF)

            if self.cz >= self.target_alt - 0.5:
                if p_state == 2:
                    self.call_sm_event(PlatformStateMachineEvent.TOOK_OFF)
                self.log("[NAV] Target altitude reached. Flying fast toward survivors' zone...")
                self.state = "GO_TO_ZONE"

        elif self.state == "GO_TO_ZONE":
            if self.platform_info.status.state == 2 and self.ticks - self.last_srv_tick >= 10:
                self.last_srv_tick = self.ticks
                self.call_sm_event(PlatformStateMachineEvent.TOOK_OFF)

            vx, vy, vz, az = compute_enu_twist(
                self.cx, self.cy, self.cz, self.yaw,
                self.center_x, self.center_y, self.target_alt,
                max_speed=OUTBOUND_SPEED, brake_gain=0.55, min_speed=0.25
            )
            self.publish_twist(vx, vy, vz, az)

            if dist_2d(self.cx, self.cy, self.center_x, self.center_y) < TRANSIT_TOLERANCE:
                self.log("[SEARCH] Search center reached. Starting Archimedean Spiral...")
                self.r = SPIRAL_SPACING * (self.theta / (2.0 * math.pi))
                self.target_x = self.center_x + self.r * math.cos(self.theta)
                self.target_y = self.center_y + self.r * math.sin(self.theta)
                self.state = "SPIRAL"

        elif self.state == "SPIRAL":
            vx, vy, vz, az = compute_enu_twist(
                self.cx, self.cy, self.cz, self.yaw,
                self.target_x, self.target_y, self.target_alt,
                max_speed=SPIRAL_SPEED, brake_gain=None
            )
            self.publish_twist(vx, vy, vz, az)

            if len(self.rescued) >= EXPECTED_SURVIVORS or self.r > MAX_RADIUS:
                reason = (
                    f"{EXPECTED_SURVIVORS} survivors located"
                    if len(self.rescued) >= EXPECTED_SURVIVORS
                    else f"max radius ({MAX_RADIUS} m)"
                )
                self.log(f"[RETURN] Search completed ({reason}). Returning to safety boat...")
                self.state = "RETURN"
            elif dist_2d(self.cx, self.cy, self.target_x, self.target_y) < SPIRAL_TOLERANCE:
                self.theta += THETA_STEP
                self.r = SPIRAL_SPACING * (self.theta / (2.0 * math.pi))
                self.target_x = self.center_x + self.r * math.cos(self.theta)
                self.target_y = self.center_y + self.r * math.sin(self.theta)

        elif self.state == "RETURN":
            vx, vy, vz, az = compute_enu_twist(
                self.cx, self.cy, self.cz, self.yaw,
                self.start_x, self.start_y, self.target_alt,
                max_speed=RETURN_SPEED, brake_gain=0.38, min_speed=0.18
            )
            self.publish_twist(vx, vy, vz, az)

            if dist_2d(self.cx, self.cy, self.start_x, self.start_y) < ALIGN_HANDOFF_DIST:
                self.log("[ALIGN] Near safety boat. Holding position to stabilize before touchdown...")
                self.stabilize_ticks = 0
                self.state = "ALIGN_BOAT"

        elif self.state == "ALIGN_BOAT":
            self.publish_pose(self.start_x, self.start_y, self.target_alt, 0.0)

            if dist_2d(self.cx, self.cy, self.start_x, self.start_y) < LAND_TOLERANCE:
                self.stabilize_ticks += 1
                if self.stabilize_ticks >= STABILIZE_TICKS:
                    self.log("[LAND] Centered over safety boat. Descending onto deck...")
                    self.call_sm_event(PlatformStateMachineEvent.LAND)
                    self.state = "LANDING"
            else:
                self.stabilize_ticks = max(0, self.stabilize_ticks - 1)

        elif self.state == "LANDING":
            # Hold horizontal position over the boat center and descend to deck
            self.publish_pose(self.start_x, self.start_y, self.start_z - 0.10, 0.0)

            if self.ticks - self.last_srv_tick >= 10:
                self.last_srv_tick = self.ticks
                self.call_sm_event(PlatformStateMachineEvent.LAND)

            if self.cz <= self.start_z + 0.20:
                self.call_sm_event(PlatformStateMachineEvent.LANDED)
                self.call_set_bool(self.cli_arm, False)
                self.state = "FINISHED"
                self.log("\n=== FINAL RESCUE REPORT ===")
                if not self.rescued:
                    self.log("No survivors were detected during the search.")
                else:
                    for p in self.rescued:
                        self.log(
                            f"Survivor #{p['id']}: GPS = ({p['lat']:.6f}, {p['lon']:.6f}) | "
                            f"Local ENU = ({p['x']:.2f}, {p['y']:.2f}) m"
                        )


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    node = RescuePeopleDirectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()