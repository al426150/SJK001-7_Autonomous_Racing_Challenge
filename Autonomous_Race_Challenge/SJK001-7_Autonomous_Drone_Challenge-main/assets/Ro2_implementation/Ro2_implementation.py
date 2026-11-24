# pylint: disable=no-member,import-error,unused-import,broad-except
import math
import cv2
import WebGUI  # Initializes rclpy internally and enables /webgui/image_debug_* topics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from as2_msgs.msg import PlatformInfo, PlatformStateMachineEvent
from as2_msgs.srv import SetControlMode, SetPlatformStateMachineEvent
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

# Reference GPS coordinates and flight parameters
BOAT_LAT, BOAT_LON = 40.0 + 16.0 / 60.0 + 48.2 / 3600.0, -(3.0 + 49.0 / 60.0 + 3.5 / 3600.0)
SURV_LAT, SURV_LON = 40.0 + 16.0 / 60.0 + 47.23 / 3600.0, -(3.0 + 49.0 / 60.0 + 1.78 / 3600.0)
M_PER_LAT, M_PER_LON = 111132.9, 111319.5 * math.cos(math.radians(BOAT_LAT))

HZ, ALTITUDE, SPIRAL_SPACING, THETA_STEP = 30.0, 4.0, 2.8, math.radians(15)
EXPECTED_SURVIVORS, MAX_RADIUS, DUPLICATE_RADIUS, FOV_SCALE = 6, 20.0, 3.2, 1.65
OUTBOUND_SPEED, RETURN_SPEED, SPIRAL_SPEED = 5.5, 2.8, 2.8
TRANSIT_TOL, ALIGN_DIST, LAND_TOL, SPIRAL_TOL, STABILIZE_TICKS = 0.7, 3.0, 0.65, 1.15, 20
CASCADE_PATH = "/resources/exercises/rescue_people/haarcascade_frontalface_default.xml"


def detect_faces(bgr_img, cascade):
    """Detects faces across 8 angles and returns pixel offsets (dx, dy)."""
    h, w = bgr_img.shape[:2]
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

    for angle in (0, 90, 180, 270, 45, 135, 225, 315):
        rot = gray if angle == 0 else cv2.warpAffine(
            gray, cv2.getRotationMatrix2D((w // 2, h // 2), angle, 1.0), (w, h)
        )
        faces = cascade.detectMultiScale(rot, scaleFactor=1.1, minNeighbors=5, minSize=(22, 22))
        valid = [
            (x, y, fw, fh) for (x, y, fw, fh) in faces
            if 0.08 * w <= x + fw / 2.0 <= 0.92 * w and 0.08 * h <= y + fh / 2.0 <= 0.92 * h
        ]
        if valid:
            ann = bgr_img.copy() if angle == 0 else cv2.warpAffine(
                bgr_img, cv2.getRotationMatrix2D((w // 2, h // 2), angle, 1.0), (w, h)
            )
            rad = math.radians(angle)
            cos_a, sin_a = math.cos(rad), math.sin(rad)
            offsets = []
            for (x, y, fw, fh) in valid:
                cv2.rectangle(ann, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
                dx_r, dy_r = (x + fw / 2.0) - w / 2.0, (y + fh / 2.0) - h / 2.0
                offsets.append((cos_a * dx_r - sin_a * dy_r, sin_a * dx_r + cos_a * dy_r))
            return offsets, ann, cv2.cvtColor(rot, cv2.COLOR_GRAY2BGR)

    return [], bgr_img.copy(), cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


class RescuePeopleDirectNode(Node):
    def __init__(self):
        super().__init__("rescue_people_direct_node")
        self.cascade = cv2.CascadeClassifier(CASCADE_PATH)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pair, self.frame_id = None, "earth"

        self.info, self.info_ok, self.pose_ok = PlatformInfo(), False, False
        self.cx = self.cy = self.cz = self.yaw = 0.0
        self.latest_img = None

        self.state = "WAIT_CONNECTION"
        self.start_x = self.start_y = self.start_z = self.center_x = self.center_y = 0.0
        self.target_alt = ALTITUDE
        self.theta, self.r, self.target_x, self.target_y = math.radians(180), 0.0, 0.0, 0.0
        self.rescued, self.ticks, self.stab_ticks = [], 0, 0
        self.last_srv_tick = self.last_mode_tick = -30

        # Aerostack2 subscribers, publishers, and service clients
        self.create_subscription(PlatformInfo, "/drone/platform/info", self._cb_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/drone/ventral_cam/image_raw", self._cb_img, qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(PoseStamped, "/drone/motion_reference/pose", 10)
        self.pub_twist = self.create_publisher(TwistStamped, "/drone/motion_reference/twist", 10)
        self.pub_right = self.create_publisher(Image, "/webgui/image_debug_right", 10)
        self.pub_left = self.create_publisher(Image, "/webgui/image_debug_left", 10)

        self.cli_arm = self.create_client(SetBool, "/drone/set_arming_state")
        self.cli_offboard = self.create_client(SetBool, "/drone/set_offboard_mode")
        self.cli_sm = self.create_client(SetPlatformStateMachineEvent, "/drone/platform/state_machine_event")
        self.cli_ctrl = self.create_client(SetControlMode, "/drone/controller/set_control_mode")

        self.create_timer(1.0 / HZ, self.control_loop)

    def _cb_info(self, msg):
        self.info, self.info_ok = msg, True

    def _cb_img(self, msg):
        self.latest_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _update_tf_pose(self):
        pairs = [self.tf_pair] if self.tf_pair else [("earth", "drone/base_link"), ("earth", "drone"), ("map", "drone/base_link")]
        for parent, child in pairs:
            try:
                t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                self.tf_pair, self.frame_id = (parent, child), parent
                self.cx, self.cy, self.cz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
                q = t.transform.rotation
                self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.pose_ok = True
                return
            except Exception:
                continue

    def _set_ctrl_mode(self, mode, yaw_mode, frame):
        cur = self.info.current_control_mode
        if (cur.control_mode, cur.yaw_mode, cur.reference_frame) == (mode, yaw_mode, frame):
            return
        if self.ticks - self.last_mode_tick >= 5 and self.cli_ctrl.service_is_ready():
            self.last_mode_tick = self.ticks
            req = SetControlMode.Request()
            req.control_mode.control_mode, req.control_mode.yaw_mode, req.control_mode.reference_frame = mode, yaw_mode, frame
            self.cli_ctrl.call_async(req)

    def _send_pose(self, x, y, z):
        self._set_ctrl_mode(2, 1, 1)  # POSITION, YAW_ANGLE, LOCAL_ENU_FRAME
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        msg.pose.orientation.w = 1.0
        self.pub_pose.publish(msg)

    def _send_vel_to(self, tx, ty, tz, max_spd, brake_gain=None, min_spd=0.20):
        self._set_ctrl_mode(3, 2, 1)  # SPEED, YAW_SPEED, LOCAL_ENU_FRAME
        dx, dy = tx - self.cx, ty - self.cy
        dist = math.hypot(dx, dy)
        spd = min(max_spd, max(min_spd, brake_gain * dist)) if (brake_gain and dist > 1e-4) else max_spd
        vx, vy = ((dx / dist) * spd, (dy / dist) * spd) if dist > 1e-4 else (0.0, 0.0)
        vz = max(-1.5, min(1.5, 1.5 * (tz - self.cz)))
        az = max(-1.0, min(1.0, 1.5 * math.atan2(math.sin(-self.yaw), math.cos(-self.yaw))))

        msg = TwistStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), self.frame_id
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.z = vx, vy, vz, az
        self.pub_twist.publish(msg)

    def _sm_event(self, event_id):
        if self.cli_sm.service_is_ready():
            req = SetPlatformStateMachineEvent.Request()
            req.event.event = int(event_id)
            self.cli_sm.call_async(req)

    def control_loop(self):
        self.ticks += 1
        self._update_tf_pose()

        # Ventral camera processing and WebGUI visualization
        if self.latest_img is not None:
            img = self.latest_img
            if self.state == "SPIRAL":
                offsets, right_img, left_img = detect_faces(img, self.cascade)
                m_per_px = (max(self.cz, 1.0) * FOV_SCALE) / img.shape[1]
                for dx_px, dy_px in offsets:
                    fx, fy = self.cx - dy_px * m_per_px, self.cy - dx_px * m_per_px
                    if all(math.hypot(fx - p["x"], fy - p["y"]) >= DUPLICATE_RADIUS for p in self.rescued):
                        lat, lon = BOAT_LAT + (fy - self.start_y) / M_PER_LAT, BOAT_LON + (fx - self.start_x) / M_PER_LON
                        self.rescued.append({"id": len(self.rescued) + 1, "x": fx, "y": fy, "lat": lat, "lon": lon})
                        self.get_logger().info(f"[DETECTION {len(self.rescued)}/{EXPECTED_SURVIVORS}] GPS: ({lat:.6f}, {lon:.6f})")
            else:
                right_img = img.copy()
                left_img = cv2.cvtColor(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)

            cv2.putText(right_img, f"{self.state} (ROS2) | Found: {len(self.rescued)}/{EXPECTED_SURVIVORS}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            self.pub_right.publish(self.bridge.cv2_to_imgmsg(right_img, encoding="bgr8"))
            self.pub_left.publish(self.bridge.cv2_to_imgmsg(left_img, encoding="bgr8"))

        # Finite state machine
        p_state = self.info.status.state
        if self.state == "WAIT_CONNECTION" and self.info_ok and self.pose_ok:
            self.start_x, self.start_y, self.start_z = self.cx, self.cy, self.cz
            self.center_x = self.start_x + (SURV_LON - BOAT_LON) * M_PER_LON
            self.center_y = self.start_y + (SURV_LAT - BOAT_LAT) * M_PER_LAT
            self.target_alt = self.start_z + ALTITUDE
            self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            self._send_pose(self.start_x, self.start_y, self.target_alt)
            if self.ticks - self.last_srv_tick >= 10:
                self.last_srv_tick = self.ticks
                if not self.info.armed and self.cli_arm.service_is_ready():
                    self.cli_arm.call_async(SetBool.Request(data=True))
                if not self.info.offboard and self.cli_offboard.service_is_ready():
                    self.cli_offboard.call_async(SetBool.Request(data=True))
                ev_map = {0: PlatformStateMachineEvent.ARM, 1: PlatformStateMachineEvent.TAKE_OFF, 2: PlatformStateMachineEvent.TOOK_OFF}
                if p_state in ev_map:
                    self._sm_event(ev_map[p_state])
            if self.cz >= self.target_alt - 0.5:
                if p_state == 2:
                    self._sm_event(PlatformStateMachineEvent.TOOK_OFF)
                self.state = "GO_TO_ZONE"

        elif self.state == "GO_TO_ZONE":
            self._send_vel_to(self.center_x, self.center_y, self.target_alt, OUTBOUND_SPEED, brake_gain=0.55, min_spd=0.25)
            if math.hypot(self.center_x - self.cx, self.center_y - self.cy) < TRANSIT_TOL:
                self.r = SPIRAL_SPACING * (self.theta / (2.0 * math.pi))
                self.target_x, self.target_y = self.center_x + self.r * math.cos(self.theta), self.center_y + self.r * math.sin(self.theta)
                self.state = "SPIRAL"

        elif self.state == "SPIRAL":
            self._send_vel_to(self.target_x, self.target_y, self.target_alt, SPIRAL_SPEED)
            if len(self.rescued) >= EXPECTED_SURVIVORS or self.r > MAX_RADIUS:
                self.state = "RETURN"
            elif math.hypot(self.target_x - self.cx, self.target_y - self.cy) < SPIRAL_TOL:
                self.theta += THETA_STEP
                self.r = SPIRAL_SPACING * (self.theta / (2.0 * math.pi))
                self.target_x, self.target_y = self.center_x + self.r * math.cos(self.theta), self.center_y + self.r * math.sin(self.theta)

        elif self.state == "RETURN":
            self._send_vel_to(self.start_x, self.start_y, self.target_alt, RETURN_SPEED, brake_gain=0.38, min_spd=0.18)
            if math.hypot(self.start_x - self.cx, self.start_y - self.cy) < ALIGN_DIST:
                self.stab_ticks, self.state = 0, "ALIGN_BOAT"

        elif self.state == "ALIGN_BOAT":
            self._send_pose(self.start_x, self.start_y, self.target_alt)
            self.stab_ticks = self.stab_ticks + 1 if math.hypot(self.start_x - self.cx, self.start_y - self.cy) < LAND_TOL else max(0, self.stab_ticks - 1)
            if self.stab_ticks >= STABILIZE_TICKS:
                self._sm_event(PlatformStateMachineEvent.LAND)
                self.state = "LANDING"

        elif self.state == "LANDING":
            self._send_pose(self.start_x, self.start_y, self.start_z - 0.10)
            if self.ticks - self.last_srv_tick >= 10:
                self.last_srv_tick = self.ticks
                self._sm_event(PlatformStateMachineEvent.LAND)
            if self.cz <= self.start_z + 0.20:
                self._sm_event(PlatformStateMachineEvent.LANDED)
                if self.cli_arm.service_is_ready():
                    self.cli_arm.call_async(SetBool.Request(data=False))
                self.state = "FINISHED"
                self.get_logger().info("\n=== FINAL RESCUE REPORT ===")
                for p in self.rescued:
                    self.get_logger().info(f"Survivor #{p['id']}: GPS = ({p['lat']:.6f}, {p['lon']:.6f}) | Local ENU = ({p['x']:.2f}, {p['y']:.2f}) m")


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