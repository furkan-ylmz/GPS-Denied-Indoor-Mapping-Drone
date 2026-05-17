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
import struct
import heapq
from enum import Enum, auto

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

import tf2_ros

try:
    import tf2_geometry_msgs  # noqa: F401
except ImportError:
    pass


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
        self.declare_parameter("position_threshold", 0.15)
        self.declare_parameter("waypoint_spacing", 0.4)
        self.declare_parameter("setpoint_count_before_offboard", 20)
        self.declare_parameter("nav_progress_timeout", 16.5)
        self.declare_parameter("nav_progress_min_delta", 0.12)
        self.declare_parameter("scan_yaw_rate", 0.4)   # rad/s — scan dönüş hızı
        self.declare_parameter("nav_yaw_rate", 1.2)     # rad/s — navigasyon yaw dönüş hızı

        self.takeoff_height = self.get_parameter("takeoff_height").value
        self.pos_threshold = self.get_parameter("position_threshold").value
        self.wp_spacing = self.get_parameter("waypoint_spacing").value
        self.setpoint_threshold = self.get_parameter(
            "setpoint_count_before_offboard").value
        self.nav_progress_timeout = float(
            self.get_parameter("nav_progress_timeout").value)
        self.nav_progress_min_delta = float(
            self.get_parameter("nav_progress_min_delta").value)
        self.scan_yaw_rate = float(
            self.get_parameter("scan_yaw_rate").value)
        self.nav_yaw_rate = float(
            self.get_parameter("nav_yaw_rate").value)

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

        # ── 3D Point Cloud subscriber (path planning için) ──
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            PointCloud2, "/drone/lidar/points", self._lidar_cb, lidar_qos)
        self.voxel_size = 0.2  # 0.2m (frontier_explorer ile aynı)
        self.voxels = {}  # (ix,iy,iz) -> True (occupied)
        self.last_scan_time = 0.0

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
        self.initial_scan_done = False  # İlk takeoff scan'i yapıldı mı?
        self.scan_incremental_yaw = 0.0  # Kademeli scan dönüşünde anlık hedef yaw
        self.smoothed_nav_yaw = float("nan")  # Navigasyon sırasında yumuşatılmış yaw

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
    #  Point Cloud & Voxel
    # ════════════════════════════════════════════

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _lidar_cb(self, msg: PointCloud2):
        """Canlı LiDAR verisinden kalıcı 3D engel haritası oluştur."""
        now = self._now_sec()
        if now - self.last_scan_time < 0.2:  # 5Hz'de güncelle (performans için)
            return
        self.last_scan_time = now

        point_step = msg.point_step
        data = bytes(msg.data)
        n = msg.width * msg.height
        offsets = {}
        for f in msg.fields:
            if f.name in ('x', 'y', 'z'):
                offsets[f.name] = f.offset
        if len(offsets) < 3:
            return
        
        # 1. Parse point cloud (lidar_link frame)
        pts_lidar = np.zeros((n, 3), dtype=np.float32)
        for i in range(n):
            base = i * point_step
            if base + offsets['z'] + 4 > len(data):
                break
            pts_lidar[i, 0] = struct.unpack_from('f', data, base + offsets['x'])[0]
            pts_lidar[i, 1] = struct.unpack_from('f', data, base + offsets['y'])[0]
            pts_lidar[i, 2] = struct.unpack_from('f', data, base + offsets['z'])[0]

        # 2. Transform lidar_link -> map
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tz = tf.transform.translation.z
            q = tf.transform.rotation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            
            R = np.array([
                [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
                [    2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
                [    2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
            ])
            
            pts_map = np.dot(pts_lidar, R.T) + np.array([tx, ty, tz])
            
            # 3. Engelleri kalıcı voxel haritasına ekle
            vs = self.voxel_size
            for i in range(0, len(pts_map), 4):  # Downsample to save CPU
                px, py, pz = pts_map[i]
                if np.isnan(px):
                    continue
                key = (int(math.floor(px/vs)), int(math.floor(py/vs)), int(math.floor(pz/vs)))
                self.voxels[key] = True

            # 4. Sürekli Engel Kontrolü (Continuous Path Checking)
            self._check_path_validity()

        except Exception as e:
            pass

    def _check_path_validity(self):
        """Eğer drone ilerlerken yolun üzerine yeni bir engel çıkarsa, anında dur ve hedefi iptal et."""
        if self.state != State.NAVIGATING or not self.waypoints_ned:
            return
        
        # Sadece önümüzdeki birkaç waypoint'i (yakın gelecek) kontrol et
        vs = self.voxel_size
        check_limit = min(self.wp_index + 4, len(self.waypoints_ned))
        for i in range(self.wp_index, check_limit):
            wp = self.waypoints_ned[i]
            x_enu, y_enu, z_enu = wp[1], wp[0], -wp[2]
            ix, iy, iz = int(math.floor(x_enu/vs)), int(math.floor(y_enu/vs)), int(math.floor(z_enu/vs))
            if not self._is_voxel_free(ix, iy, iz):
                self.get_logger().warn("Yol ustunde anlik engel tespit edildi! Yeniden planlanacak.")
                self.state = State.HOVER
                self._publish_nav_status("REPLAN")
                return

    def _is_voxel_free(self, ix, iy, iz):
        """Voxel ve komşuları engel içermiyor mu kontrol et. (Silindirik Padding)"""
        padding_z = 1   # 1 voxel * 0.2 = 0.2m (Toplam Yükseklik 40cm)
        
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                # Euclidean distance check (40cm radius)
                dist_xy = math.sqrt(dx*dx + dy*dy) * self.voxel_size
                if dist_xy > 0.42:  # 0.4m yarıçap + ufak tolerans
                    continue
                for dz in range(-padding_z, padding_z + 1):
                    if (ix+dx, iy+dy, iz+dz) in self.voxels:
                        return False
        return True

    def _get_nearest_free_voxel(self, vx, vy, vz):
        """Voxel doluysa en yakın boş voxeli bul (A* için)."""
        if self._is_voxel_free(vx, vy, vz):
            return (vx, vy, vz)
        for r in range(1, 6):  # 5 voxel yarıçapına kadar ara (1.0m)
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    for dz in range(-1, 2):
                        if self._is_voxel_free(vx + dx, vy + dy, vz + dz):
                            return (vx + dx, vy + dy, vz + dz)
        return None

    # ════════════════════════════════════════════
    #  3D Path Planning (A*)
    # ════════════════════════════════════════════

    def _plan_path(self, goal_pose: PoseStamped):
        """3D A* ile engelden kaçınan path hesapla."""
        if self.planning_in_progress:
            self.get_logger().info("Onceki plan iptal, yeni plan isteniyor...")

        if not self.voxels:
            self.get_logger().warn("Voxel haritasi bos -> HOVER'da bekleniyor")
            self.planning_in_progress = False
            self._publish_nav_status("FAILED")
            return

        # Robot pozisyonu (ENU): NED->ENU
        rx_enu = self.pos_ned[1]
        ry_enu = self.pos_ned[0]
        rz_enu = -self.pos_ned[2]

        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y
        gz = goal_pose.pose.position.z if goal_pose.pose.position.z != 0.0 else rz_enu

        vs = self.voxel_size
        start = (int(math.floor(rx_enu/vs)), int(math.floor(ry_enu/vs)), int(math.floor(rz_enu/vs)))
        goal = (int(math.floor(gx/vs)), int(math.floor(gy/vs)), int(math.floor(gz/vs)))

        # Başlangıç veya hedef padding yüzünden "dolu" görünüyorsa en yakın boş voxele kaydır
        start = self._get_nearest_free_voxel(*start)
        goal = self._get_nearest_free_voxel(*goal)

        if not start or not goal:
            self.get_logger().warn("Start veya Goal etrafi tamamen dolu! -> HOVER")
            self.planning_in_progress = False
            self._publish_nav_status("FAILED")
            return

        path_voxels = self._astar_3d(start, goal)

        if path_voxels is None:
            self.get_logger().warn("3D A* path bulunamadi -> HOVER (Hedef iptal)")
            self.planning_in_progress = False
            self._publish_nav_status("FAILED")
            return

        # Voxel path -> map frame PoseStamped path (RViz görselleştirme)
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for vx, vy, vz in path_voxels:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = (vx + 0.5) * vs
            ps.pose.position.y = (vy + 0.5) * vs
            ps.pose.position.z = (vz + 0.5) * vs
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

        # Map ENU path -> NED waypoints
        ned_wps = []
        last_wp = None
        for vx, vy, vz in path_voxels:
            wx = (vx + 0.5) * vs  # ENU x
            wy = (vy + 0.5) * vs  # ENU y
            wz = (vz + 0.5) * vs  # ENU z
            # ENU -> NED
            wp = [wy, wx, -wz]  # NED: north=enu_y, east=enu_x, down=-enu_z
            if last_wp is not None:
                dist = math.sqrt((wp[0]-last_wp[0])**2 + (wp[1]-last_wp[1])**2)
                if dist < self.wp_spacing:
                    continue
            ned_wps.append(wp)
            last_wp = wp

        # Son noktayı ekle
        final = [gy, gx, -gz]
        if not ned_wps or ned_wps[-1] != final:
            ned_wps.append(final)

        if not ned_wps:
            self.get_logger().warn("Path donusumu bos — HOVER'da bekleniyor")
            return

        self.waypoints_ned = ned_wps
        self.wp_index = 0
        self.target_ned = self.waypoints_ned[0]
        self.state = State.NAVIGATING
        self._reset_nav_progress_watchdog()
        self._publish_nav_status("NAVIGATING")
        self.planning_in_progress = False
        self.get_logger().info(
            f"3D navigasyon basladi: {len(self.waypoints_ned)} waypoint")

    def _astar_3d(self, start, goal, max_iter=100000):
        """3D A* path planning on voxel grid. Weighted for speed."""
        def heuristic(a, b):
            return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

        # 26-connected neighbors (full 3D)
        neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    neighbors.append((dx, dy, dz))

        open_set = [(heuristic(start, goal), 0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        closed = set()
        iterations = 0

        while open_set and iterations < max_iter:
            iterations += 1
            f, g, current = heapq.heappop(open_set)
            if current in closed:
                continue
            closed.add(current)

            # Goal check (1 voxel tolerance)
            if abs(current[0]-goal[0]) <= 1 and abs(current[1]-goal[1]) <= 1 and abs(current[2]-goal[2]) <= 1:
                # Reconstruct path
                path = [goal, current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            cx, cy, cz = current
            for dx, dy, dz in neighbors:
                nx, ny, nz = cx+dx, cy+dy, cz+dz
                if (nx, ny, nz) in closed:
                    continue
                if not self._is_voxel_free(nx, ny, nz):
                    continue
                move_cost = math.sqrt(dx*dx + dy*dy + dz*dz)
                ng = g + move_cost
                if ng < g_score.get((nx, ny, nz), float('inf')):
                    g_score[(nx, ny, nz)] = ng
                    came_from[(nx, ny, nz)] = (cx, cy, cz)
                    # Weighted A* (W=1.5) for much faster planning
                    priority = ng + 1.5 * heuristic((nx, ny, nz), goal)
                    heapq.heappush(open_set, (priority, ng, (nx, ny, nz)))

        return None  # Path not found



    def _set_direct_goal(self, goal_pose: PoseStamped):
        """Engel bilgisi olmadan direkt hedefe git (fallback)."""
        x_enu = goal_pose.pose.position.x
        y_enu = goal_pose.pose.position.y
        z_enu = goal_pose.pose.position.z if goal_pose.pose.position.z != 0.0 else self.takeoff_height

        wp = [y_enu, x_enu, -z_enu]  # ENU → NED
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
        current_tolerance = self.pos_threshold if is_final_wp else 0.2
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

        # ── Durum bazlı yaw hesaplama ──
        loop_yaw = float("nan")
        if self.state == State.SCANNING:
            loop_yaw = self.scan_incremental_yaw
        elif self.state == State.NAVIGATING:
            # Drone'un yüzünü hedefe çevir — kademeli (savrulmayı önler)
            dx = self.target_ned[0] - self.pos_ned[0]  # North farkı
            dy = self.target_ned[1] - self.pos_ned[1]  # East farkı
            if math.hypot(dx, dy) > 0.3:
                desired_yaw = math.atan2(dy, dx)  # NED'de hedef yaw

                # İlk seferde mevcut yaw'dan başla
                if math.isnan(self.smoothed_nav_yaw):
                    self.smoothed_nav_yaw = self.current_yaw

                # Kademeli interpolasyon: nav_yaw_rate rad/s, 20Hz tick
                yaw_diff = self._wrap_pi(desired_yaw - self.smoothed_nav_yaw)
                max_step = self.nav_yaw_rate * 0.05  # 0.05s = 1 tick

                if abs(yaw_diff) <= max_step:
                    self.smoothed_nav_yaw = desired_yaw
                else:
                    step = max_step if yaw_diff > 0 else -max_step
                    self.smoothed_nav_yaw = self._wrap_pi(
                        self.smoothed_nav_yaw + step)

                loop_yaw = self.smoothed_nav_yaw

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
                if not self.initial_scan_done:
                    self.get_logger().info(
                        f"Takeoff tamam | z={-self.pos_ned[2]:.2f}m -> ilk SCANNING")
                    self.target_yaw = self._wrap_pi(self.current_yaw + math.pi)
                    self.scan_incremental_yaw = self.current_yaw
                    self.state = State.SCANNING
                else:
                    self.get_logger().info(
                        f"Takeoff tamam | z={-self.pos_ned[2]:.2f}m -> HOVER")
                    self.state = State.HOVER

        # ── SCANNING: Etrafı haritalamak için kademeli yaw dönüşü ──
        # Yaw dönüşü yavaş yapılır (harita kaymasını önlemek için)
        elif self.state == State.SCANNING:
            dyaw = self._wrap_pi(self.target_yaw - self.current_yaw)
            if abs(dyaw) < 0.15:  # ~8.5 derece tolerans
                self.initial_scan_done = True
                self.get_logger().info("Kademeli dönüs tamamlandi -> HOVER")
                self.state = State.HOVER
                self._publish_nav_status("REACHED")
                # Kuyrukta daha onceden gelmis bir hedef varsa planla
                if self.goal_pose_map is not None:
                    self._plan_path(self.goal_pose_map)
            else:
                # Kademeli yaw artışı: scan_yaw_rate rad/s, 20Hz tick
                yaw_step = self.scan_yaw_rate * 0.05  # 0.05s = 1 tick
                if dyaw > 0:
                    self.scan_incremental_yaw = self._wrap_pi(
                        self.scan_incremental_yaw + yaw_step)
                else:
                    self.scan_incremental_yaw = self._wrap_pi(
                        self.scan_incremental_yaw - yaw_step)

        # ── HOVER: Yerinde dur, hedef bekle ──
        elif self.state == State.HOVER:
            pass  # /goal_pose callback'i ile navigasyona geçilir

        # ── NAVIGATING: Waypoint'leri sırayla takip et ──
        # Drone yüzünü hedefe döndürerek ilerler (yaw üstte hesaplandı)
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
                    # Hedefe ulaşıldı — artık SCANNING yerine direkt HOVER
                    # (ilk takeoff scan'i zaten yapıldı, gezme sırasında
                    #  LiDAR sürekli harita oluşturuyor)
                    self.get_logger().info("Hedefe ulasildi! -> HOVER")
                    self.goal_pose_map = None
                    self.state = State.HOVER
                    self._clear_nav_progress_watchdog()
                    self.smoothed_nav_yaw = float("nan")
                    self._publish_nav_status("REACHED")
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
                self.smoothed_nav_yaw = float("nan")
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
