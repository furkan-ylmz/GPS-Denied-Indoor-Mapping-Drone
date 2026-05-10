"""
Drone Navigator — Nav2 Path Planning + PX4 Otonom Navigasyon
═══════════════════════════════════════════════════════════════

RViz2'den /goal_pose ile hedef alır, Nav2 planner ile engelden
sakınmalı path hesaplar, PX4 position setpoint ile waypoint'leri takip eder.

Özellikler:
  - Otomatik arm + takeoff + hover
  - RViz2'den /goal_pose ile hedef belirleme
  - Nav2 ComputePathToPose ile global path planning
  - PX4 position setpoint (NED) ile path takibi
  - TF (map → odom) ile doğru koordinat dönüşümü
  - Nav2 yoksa direkt hedefe gitme (fallback)
  - /drone/planned_path ile RViz2'de path görselleştirme

Koordinat Dönüşümü:
  Nav2 path (map frame, ENU) → TF → odom frame (ENU) → NED → PX4 setpoint
  ENU → NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu

Kullanım:
  # Nav2 + SLAM çalışıyor olmalı
  ros2 run px4_offboard drone_navigator
  ros2 run px4_offboard drone_navigator --ros-args -p takeoff_height:=2.0

  # RViz2'de "2D Nav Goal" butonu ile hedef belirle
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

import tf2_ros

# tf2_geometry_msgs'i import etmek PoseStamped için TF2 dönüşüm desteği sağlar
try:
    import tf2_geometry_msgs  # noqa: F401
except ImportError:
    pass

from nav2_msgs.action import ComputePathToPose


# ── QoS: PX4 uXRCE-DDS best-effort stream ile uyumlu ──
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class State(Enum):
    """Drone durum makinesi."""
    INIT = auto()
    OFFBOARD_REQ = auto()
    ARMING = auto()
    TAKEOFF = auto()
    HOVER = auto()
    NAVIGATING = auto()
    SCANNING = auto()


class DroneNavigator(Node):
    def __init__(self):
        super().__init__("drone_navigator")

        self.current_yaw = 0.0
        self.target_yaw = 0.0

        # ── Parametreler ──
        self.declare_parameter("takeoff_height", 1.0)
        self.declare_parameter("position_threshold", 0.22)
        self.declare_parameter("waypoint_spacing", 0.8)
        self.declare_parameter("setpoint_count_before_offboard", 20)
        self.declare_parameter("nav_progress_timeout", 6.5)
        self.declare_parameter("nav_progress_min_delta", 0.12)

        self.takeoff_height = self.get_parameter("takeoff_height").value
        self.pos_threshold = self.get_parameter("position_threshold").value
        self.wp_spacing = self.get_parameter("waypoint_spacing").value
        self.setpoint_threshold = self.get_parameter(
            "setpoint_count_before_offboard").value
        self.nav_progress_timeout = float(
            self.get_parameter("nav_progress_timeout").value)
        self.nav_progress_min_delta = float(
            self.get_parameter("nav_progress_min_delta").value)

        # ── PX4 Publishers ──
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)

        # ── Görselleştirme Publisher ──
        self.path_pub = self.create_publisher(Path, "/drone/planned_path", 10)

        # ── Navigator durum publisher (explorer için) ──
        self.nav_status_pub = self.create_publisher(
            String, "/navigator/status", 10)

        # ── PX4 Subscribers ──
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
            self._pos_cb, PX4_QOS)
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v2",
            self._status_cb, PX4_QOS)

        # ── Goal Subscriber (RViz2 "2D Nav Goal") ──
        self.create_subscription(
            PoseStamped, "/goal_pose", self._goal_cb, 10)

        # ── TF2 ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Nav2 Planner Action Client ──
        self.planner_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose")
        self.planner_available = False

        # ── Durum ──
        self.state = State.INIT
        self.counter = 0
        self.pos_ned = [0.0, 0.0, 0.0]
        self.armed = False
        self.nav_state = 0
        self.target_ned = [0.0, 0.0, -self.takeoff_height]

        # ── Navigasyon ──
        self.waypoints_ned = []
        self.wp_index = 0
        self.goal_pose_map = None  # Kuyrukta bekleyen hedef (henüz hover'da değilken)
        self.planning_in_progress = False
        self.nav_progress_best_dist = None
        self.nav_progress_last_improve_time = None

        # ── 20 Hz kontrol döngüsü ──
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f"Navigator baslatildi | takeoff: {self.takeoff_height}m | "
            f"RViz2'de '2D Nav Goal' ile hedef belirleyin")

    # ════════════════════════════════════════════
    #  Callbacks
    # ════════════════════════════════════════════

    def _pos_cb(self, msg: VehicleLocalPosition):
        self.pos_ned = [msg.x, msg.y, msg.z]
        self.current_yaw = msg.heading

    def _status_cb(self, msg: VehicleStatus):
        old_armed = self.armed
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state
        if self.armed and not old_armed:
            self.get_logger().info("ARM edildi")
        if not self.armed and old_armed:
            self.get_logger().info("DISARM edildi")

    def _goal_cb(self, msg: PoseStamped):
        """RViz2'den /goal_pose geldiğinde."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(
            f"Yeni hedef: x={x:.2f}, y={y:.2f} (map frame, ENU)")

        self.goal_pose_map = msg

        if self.state not in (State.HOVER, State.NAVIGATING):
            self.get_logger().warn(
                "Drone henuz hover'da degil — hedef beklemeye alindi")
            return

        self._plan_path(msg)

    # ════════════════════════════════════════════
    #  Path Planning (Nav2)
    # ════════════════════════════════════════════

    def _plan_path(self, goal_pose: PoseStamped):
        """Nav2 planner'dan path iste, yoksa direkt git."""
        if self.planning_in_progress:
            self.get_logger().info("Onceki plan iptal, yeni plan isteniyor...")

        # Nav2 planner kontrolü
        if not self.planner_available:
            self.planner_available = self.planner_client.wait_for_server(
                timeout_sec=1.0)

        if self.planner_available:
            self.get_logger().info("Nav2 planner'dan path isteniyor...")
            self.planning_in_progress = True

            goal_msg = ComputePathToPose.Goal()
            goal_msg.goal = goal_pose
            goal_msg.use_start = False  # Robot'un mevcut konumunu kullan

            future = self.planner_client.send_goal_async(goal_msg)
            future.add_done_callback(self._on_plan_accepted)
        else:
            self.get_logger().warn(
                "Nav2 planner bulunamadi -> direkt hedefe gidiliyor")
            self._set_direct_goal(goal_pose)

    def _on_plan_accepted(self, future):
        """Planner goal kabul/red cevabı."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                "Path istegi reddedildi — HOVER'da bekleniyor")
            self.planning_in_progress = False
            self._publish_nav_status("FAILED")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future):
        """Planner sonucu: path veya hata."""
        self.planning_in_progress = False
        result = future.result().result
        path = result.path

        if len(path.poses) < 2:
            # Tanılama: neden path bulunamadı?
            gp = self.goal_pose_map
            goal_str = ""
            if gp is not None:
                goal_str = (f" | hedef: ({gp.pose.position.x:.1f}, "
                            f"{gp.pose.position.y:.1f})")
            robot_enu_x = self.pos_ned[1]  # NED→ENU
            robot_enu_y = self.pos_ned[0]
            self.get_logger().warn(
                f"Path bulunamadi! robot: ({robot_enu_x:.1f}, "
                f"{robot_enu_y:.1f}){goal_str} | "
                f"poses: {len(path.poses)}")
            self._publish_nav_status("FAILED")
            return

        self.get_logger().info(
            f"Path alindi: {len(path.poses)} nokta | "
            f"sure: {result.planning_time.sec}.{result.planning_time.nanosec//1000000:03d}s")

        # İlk ve son noktayı logla (tanılama)
        p0 = path.poses[0].pose.position
        pN = path.poses[-1].pose.position
        self.get_logger().info(
            f"  start: ({p0.x:.1f}, {p0.y:.1f}) | "
            f"end: ({pN.x:.1f}, {pN.y:.1f})")

        # RViz2'de path'i göster
        self.path_pub.publish(path)

        # Map frame path → NED waypoint listesi
        ned_wps = self._convert_path_to_ned(path)

        if not ned_wps:
            self.get_logger().warn("Path donusumu bos — HOVER'da bekleniyor")
            return

        self.waypoints_ned = ned_wps
        self.wp_index = 0
        self.target_ned = self.waypoints_ned[0]
        self.state = State.NAVIGATING
        self._reset_nav_progress_watchdog()
        self._publish_nav_status("NAVIGATING")
        self.get_logger().info(
            f"Navigasyon basladi: {len(self.waypoints_ned)} waypoint")

    # ════════════════════════════════════════════
    #  Koordinat Dönüşümleri
    # ════════════════════════════════════════════

    def _convert_path_to_ned(self, path: Path):
        """Map frame path → NED waypoint listesi.

        1. TF ile map → odom (ENU) dönüşümü (1 kere sorgulanır)
        2. ENU → NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
        3. Z sabit (takeoff yüksekliği)
        4. Yakın waypoint'leri birleştir (downsample)
        """
        ned_waypoints = []
        last_wp = None

        # TF mevcut mu? Sadece 1 kere sorgula!
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'map', rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
        except Exception:
            self.get_logger().warn(
                "TF map->odom bulunamadi, direkt donusum kullaniliyor")

        for pose_s in path.poses:
            x_enu, y_enu = self._fast_pose_to_odom_enu(pose_s, transform)

            # ENU → NED
            x_ned = y_enu    # North = ENU_y
            y_ned = x_enu    # East  = ENU_x
            z_ned = -self.takeoff_height  # Sabit irtifa

            wp = [x_ned, y_ned, z_ned]

            # Downsample: çok yakın noktaları atla
            if last_wp is not None:
                dist = math.sqrt(
                    (wp[0] - last_wp[0]) ** 2 + (wp[1] - last_wp[1]) ** 2)
                if dist < self.wp_spacing:
                    continue

            ned_waypoints.append(wp)
            last_wp = wp

        # Son waypoint'i her zaman ekle
        if path.poses:
            fx, fy = self._fast_pose_to_odom_enu(path.poses[-1], transform)
            final_wp = [fy, fx, -self.takeoff_height]
            if not ned_waypoints or ned_waypoints[-1] != final_wp:
                ned_waypoints.append(final_wp)

        return ned_waypoints

    def _fast_pose_to_odom_enu(self, pose_s: PoseStamped, transform):
        """PoseStamped (map frame) → odom ENU (x, y) pozisyonu. (Hızlı çeviri)"""
        if transform is not None:
            try:
                import tf2_geometry_msgs
                odom_pose = tf2_geometry_msgs.do_transform_pose(pose_s.pose, transform)
                return odom_pose.position.x, odom_pose.position.y
            except Exception:
                pass
        # Fallback: map ≈ odom varsay
        return pose_s.pose.position.x, pose_s.pose.position.y

    def _pose_to_odom_enu(self, pose_s: PoseStamped, use_tf: bool):
        """Eski yavaş dönüşüm (geriye dönük uyumluluk için saklanabilir veya kullanılabilir)."""
        if use_tf:
            try:
                odom_pose = self.tf_buffer.transform(
                    pose_s, 'odom', timeout=Duration(seconds=0.2))
                return odom_pose.pose.position.x, odom_pose.pose.position.y
            except Exception:
                pass
        # Fallback: map ≈ odom varsay
        return pose_s.pose.position.x, pose_s.pose.position.y

    def _set_direct_goal(self, goal_pose: PoseStamped):
        """Nav2 olmadan direkt hedefe git (fallback)."""
        x_enu, y_enu = self._pose_to_odom_enu(goal_pose, use_tf=True)

        wp = [y_enu, x_enu, -self.takeoff_height]  # ENU → NED
        self.waypoints_ned = [wp]
        self.wp_index = 0
        self.target_ned = wp

        if self.state in (State.HOVER, State.NAVIGATING):
            self.state = State.NAVIGATING
            self._reset_nav_progress_watchdog()
            self.get_logger().info(
                f"Direkt hedef: NED ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f})")

        # RViz2'de düz çizgi path göster
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        start = PoseStamped()
        start.header = path_msg.header
        start.pose.position.x = float(self.pos_ned[1])  # NED → ENU
        start.pose.position.y = float(self.pos_ned[0])
        start.pose.position.z = float(-self.pos_ned[2])

        path_msg.poses = [start, goal_pose]
        self.path_pub.publish(path_msg)

    # ════════════════════════════════════════════
    #  PX4 Komutları
    # ════════════════════════════════════════════

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._ts()
        self.offboard_pub.publish(msg)

    def _publish_setpoint(self, x, y, z, yaw=float("nan")):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float("nan"), float("nan"), float("nan")]
        msg.acceleration = [float("nan"), float("nan"), float("nan")]
        msg.yaw = float(yaw)
        msg.timestamp = self._ts()
        self.setpoint_pub.publish(msg)

    def _send_command(self, cmd, p1=0.0, p2=0.0, p7=0.0):
        msg = VehicleCommand()
        msg.command = cmd
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.param7 = float(p7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._ts()
        self.command_pub.publish(msg)

    def _ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _wrap_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # ════════════════════════════════════════════
    #  Yardımcılar
    # ════════════════════════════════════════════

    def _dist_to_target_2d(self):
        return math.sqrt((self.pos_ned[0] - self.target_ned[0]) ** 2 + (self.pos_ned[1] - self.target_ned[1]) ** 2)

    def _dist_to_target_z(self):
        return abs(self.pos_ned[2] - self.target_ned[2])

    def _at_target_2d(self):
        is_final_wp = (self.wp_index >= len(self.waypoints_ned) - 1)
        current_tolerance = self.pos_threshold if is_final_wp else 0.8
        return self._dist_to_target_2d() < current_tolerance

    def _at_target_z(self):
        return self._dist_to_target_z() < self.pos_threshold

    def _publish_nav_status(self, status):
        """Navigator durumunu yayınla (NAVIGATING/REACHED/FAILED)."""
        msg = String()
        msg.data = status
        self.nav_status_pub.publish(msg)

    def _reset_nav_progress_watchdog(self):
        self.nav_progress_best_dist = self._dist_to_target_2d()
        self.nav_progress_last_improve_time = self.get_clock().now()

    def _clear_nav_progress_watchdog(self):
        self.nav_progress_best_dist = None
        self.nav_progress_last_improve_time = None

    def _nav_progress_stalled(self):
        now = self.get_clock().now()
        dist = self._dist_to_target_2d()

        if (
            self.nav_progress_best_dist is None
            or self.nav_progress_last_improve_time is None
        ):
            self.nav_progress_best_dist = dist
            self.nav_progress_last_improve_time = now
            return False

        if dist < (self.nav_progress_best_dist - self.nav_progress_min_delta):
            self.nav_progress_best_dist = dist
            self.nav_progress_last_improve_time = now
            return False

        elapsed = (now - self.nav_progress_last_improve_time).nanoseconds / 1e9
        return elapsed > self.nav_progress_timeout

    # ════════════════════════════════════════════
    #  Ana Kontrol Döngüsü (20Hz)
    # ════════════════════════════════════════════

    def _control_loop(self):
        # Her döngüde offboard mode + setpoint gönder
        self._publish_offboard_mode()
        
        loop_yaw = float("nan")
        if self.state == State.SCANNING:
            loop_yaw = self.target_yaw

        self._publish_setpoint(*self.target_ned, yaw=loop_yaw)

        # ── INIT: Yeterli setpoint gönder ──
        if self.state == State.INIT:
            self.counter += 1
            if self.counter % 20 == 0:
                self.get_logger().info(
                    f"[INIT] Setpoint: {self.counter}/{self.setpoint_threshold}")
            if self.counter >= self.setpoint_threshold:
                self.get_logger().info("OFFBOARD mod komutu gonderiliyor...")
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.state = State.OFFBOARD_REQ
                self.counter = 0

        # ── OFFBOARD_REQ: Offboard mod aktif olmasını bekle ──
        elif self.state == State.OFFBOARD_REQ:
            self.counter += 1
            if self.nav_state == 14:  # NAVIGATION_STATE_OFFBOARD
                self.get_logger().info("OFFBOARD aktif -> ARM gonderiliyor")
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.state = State.ARMING
                self.counter = 0
            elif self.counter % 20 == 0:
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self.counter > 200:
                self.get_logger().error(
                    "OFFBOARD moda gecilemedi! Parametreleri kontrol edin.")
                self.state = State.INIT
                self.counter = 0

        # ── ARMING: ARM olmasını bekle ──
        elif self.state == State.ARMING:
            self.counter += 1
            if self.armed:
                self.get_logger().info(
                    f"ARMED -> TAKEOFF ({self.takeoff_height}m)")
                self.state = State.TAKEOFF
                self.counter = 0
            elif self.counter % 20 == 0:
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if self.counter > 200:
                self.get_logger().error("ARM basarisiz!")
                self.state = State.INIT
                self.counter = 0

        # ── TAKEOFF: Hedef yüksekliğe ulaşana kadar bekle ──
        elif self.state == State.TAKEOFF:
            if self._at_target_z():
                self.get_logger().info(
                    f"Takeoff tamam | z={-self.pos_ned[2]:.2f}m -> SCANNING")
                self.target_yaw = self._wrap_pi(self.current_yaw + math.pi)
                self.state = State.SCANNING

        # ── SCANNING: Etrafı haritalamak için 180 derece olduğu yerde dön ──
        elif self.state == State.SCANNING:
            dyaw = self._wrap_pi(self.target_yaw - self.current_yaw)
            if abs(dyaw) < 0.15:  # ~8.5 derece tolerans
                self.get_logger().info("180 derece dönüs tamamlandi -> HOVER")
                self.state = State.HOVER
                self._publish_nav_status("REACHED")
                # Kuyrukta daha onceden gelmis bir hedef varsa planla
                if self.goal_pose_map is not None:
                    self._plan_path(self.goal_pose_map)

        # ── HOVER: Yerinde dur, hedef bekle ──
        elif self.state == State.HOVER:
            pass  # /goal_pose callback'i ile navigasyona geçilir

        # ── NAVIGATING: Waypoint'leri sırayla takip et ──
        elif self.state == State.NAVIGATING:
            if self._at_target_2d():
                self.wp_index += 1
                if self.wp_index < len(self.waypoints_ned):
                    self.target_ned = self.waypoints_ned[self.wp_index]
                    self._reset_nav_progress_watchdog()
                    remaining = len(self.waypoints_ned) - self.wp_index
                    self.get_logger().info(
                        f"WP {self.wp_index + 1}/{len(self.waypoints_ned)} | "
                        f"kalan: {remaining} | "
                        f"NED ({self.target_ned[0]:.1f}, {self.target_ned[1]:.1f})")
                else:
                    self.get_logger().info("Hedefe ulasildi! -> SCANNING")
                    self.goal_pose_map = None
                    self.target_yaw = self._wrap_pi(self.current_yaw + math.pi)
                    self.state = State.SCANNING
                    self._clear_nav_progress_watchdog()
            elif self._nav_progress_stalled():
                self.get_logger().warn(
                    "Navigasyon ilerlemiyor -> FAILED, HOVER'a donuluyor"
                )
                self.waypoints_ned = []
                self.wp_index = 0
                self.goal_pose_map = None
                self.target_ned = [
                    float(self.pos_ned[0]),
                    float(self.pos_ned[1]),
                    -float(self.takeoff_height),
                ]
                self.state = State.HOVER
                self._clear_nav_progress_watchdog()
                self._publish_nav_status("FAILED")

    def do_land(self):
        """İniş komutu gönder."""
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("LAND komutu gonderildi")


def main(args=None):
    rclpy.init(args=args)
    node = DroneNavigator()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        try:
            if rclpy.ok():
                node.get_logger().info("Ctrl+C -> inis yapiliyor...")
                node.do_land()
                import time
                time.sleep(2.0)
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
