"""
Frontier Explorer — DFS Ağaç Tabanlı Otonom Bina İçi Keşif
═══════════════════════════════════════════════════════════════

Depth-First Search (DFS) backtracking algoritması ile bina içini
sistematik olarak keşfeder.

Algoritma (DFS Ağaç):
  1. Başlangıç noktasında root node oluştur
  2. Tüm frontier'ları bul, EN UZAĞA git (derinlik öncelikli)
  3. Hedefe ulaşınca yeni node oluştur, oradan da en uzağa git
  4. Dallanma devam eder (ağaç derinleşir)
  5. Bir dalda frontier kalmazsa → bir önceki node'a geri dön (backtrack)
  6. O node'daki kalan en uzak frontier'a git
  7. Tüm node'lar tamamlanınca → keşif biter

Örnek ağaç:
     [Root] ─── en uzak ──→ [A] ─── en uzak ──→ [B] (dal bitti)
        │                    │                         ↑ backtrack
        │                    └── 2. uzak ──→ [C]      [B→A]
        └── 2. uzak ──→ [D] ─── ...                   ↑ backtrack
                                                       [A→Root]

Kullanım:
  ros2 run px4_offboard frontier_explorer
"""

import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import tf2_ros
from rclpy.duration import Duration


# OccupancyGrid değerleri
UNKNOWN = -1
FREE = 0


class ExploreNode:
    """DFS ağacındaki bir keşif noktası."""
    __slots__ = ("position", "targets", "target_idx")

    def __init__(self, position, targets):
        self.position = position         # (x, y) dünya koordinatları
        self.targets = targets           # [(x, y, size), ...] uzaktan yakına sıralı
        self.target_idx = 0              # sıradaki hedef indeksi

    @property
    def has_remaining(self):
        return self.target_idx < len(self.targets)

    @property
    def current_target(self):
        if self.has_remaining:
            t = self.targets[self.target_idx]
            return (t[0], t[1])
        return None

    def advance(self):
        """Bir sonraki hedefe geç (mevcut dal tükendiğinde)."""
        self.target_idx += 1


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        # ── Parametreler ──
        self.declare_parameter("min_frontier_size", 5)
        self.declare_parameter("goal_tolerance", 1.5)
        self.declare_parameter("blacklist_radius", 2.0)
        self.declare_parameter("update_interval", 3.0)
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("costmap_topic", "/map")
        self.declare_parameter("goal_timeout", 90.0)

        self.min_frontier_size = self.get_parameter("min_frontier_size").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.blacklist_radius = self.get_parameter("blacklist_radius").value
        self.update_interval = self.get_parameter("update_interval").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.goal_timeout = self.get_parameter("goal_timeout").value

        # ── Subscribers ──
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("costmap_topic").value,
            self._map_cb,
            map_qos,
        )

        # ── Publishers ──
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.exploring_pub = self.create_publisher(Bool, "/explorer/active", 10)

        # ── TF ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── DFS Ağaç Durumu ──
        self.explore_stack = []          # [ExploreNode, ...] DFS stack
        self.blacklisted = []            # [(x, y), ...] ulaşılamayan hedefler
        self.current_goal = None         # (x, y) aktif hedef
        self.goal_sent_time = None
        self.exploring = False
        self.backtracking = False        # Geriye dönüş modunda mı?
        self.no_frontier_count = 0
        self.max_no_frontier = 5

        # ── Harita ──
        self.map_data = None
        self.map_info = None

        # ── Timer ──
        self.timer = self.create_timer(self.update_interval, self._explore_tick)

        self.get_logger().info(
            "DFS Frontier Explorer baslatildi | "
            "Strateji: en uzak frontier oncelikli + backtracking")

    # ════════════════════════════════════════════
    #  Callbacks
    # ════════════════════════════════════════════

    def _map_cb(self, msg: OccupancyGrid):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self.map_info = msg.info

    # ════════════════════════════════════════════
    #  Koordinat Dönüşümleri
    # ════════════════════════════════════════════

    def _get_robot_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", self.robot_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None

    def _grid_to_world(self, gx, gy):
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        return (ox + (gx + 0.5) * res, oy + (gy + 0.5) * res)

    # ════════════════════════════════════════════
    #  Frontier Tespiti
    # ════════════════════════════════════════════

    def _find_frontiers(self):
        """Haritadaki frontier kümelerini bul.

        Returns: [(wx, wy, size), ...] dünya koordinatları ve küme boyutu.
        """
        if self.map_data is None or self.map_info is None:
            return []

        h, w = self.map_data.shape

        free_mask = (self.map_data == FREE)
        unknown_mask = (self.map_data == UNKNOWN)

        # 4-yönlü komşuluk ile frontier tespiti
        has_unknown_neighbor = np.zeros((h, w), dtype=bool)
        if h > 1:
            has_unknown_neighbor[1:, :] |= unknown_mask[:-1, :]
            has_unknown_neighbor[:-1, :] |= unknown_mask[1:, :]
        if w > 1:
            has_unknown_neighbor[:, 1:] |= unknown_mask[:, :-1]
            has_unknown_neighbor[:, :-1] |= unknown_mask[:, 1:]

        frontier_mask = free_mask & has_unknown_neighbor

        # BFS ile kümeleme
        frontier_coords = np.argwhere(frontier_mask)
        if len(frontier_coords) == 0:
            return []

        visited = set()
        results = []

        for row, col in frontier_coords:
            if (row, col) in visited:
                continue

            cluster_gx = []
            cluster_gy = []
            queue = deque([(row, col)])
            visited.add((row, col))

            while queue:
                r, c = queue.popleft()
                cluster_gx.append(c)
                cluster_gy.append(r)

                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r + dr, c + dc
                    if (0 <= nr < h and 0 <= nc < w and
                            (nr, nc) not in visited and frontier_mask[nr, nc]):
                        visited.add((nr, nc))
                        queue.append((nr, nc))

            size = len(cluster_gx)
            if size >= self.min_frontier_size:
                cx = sum(cluster_gx) / size
                cy = sum(cluster_gy) / size
                wx, wy = self._grid_to_world(cx, cy)
                results.append((wx, wy, size))

        return results

    # ════════════════════════════════════════════
    #  DFS Ağaç: Node Oluşturma
    # ════════════════════════════════════════════

    def _create_explore_node(self, position, frontiers):
        """Mevcut konumda yeni DFS node'u oluştur.

        Frontier'ları mesafeye göre UZAKTAN YAKINA sıralar.
        İlk hedef = en uzak frontier (DFS derinlik öncelikli).
        """
        rx, ry = position

        # Blacklist filtrele ve mesafe hesapla
        scored = []
        for wx, wy, size in frontiers:
            if self._is_blacklisted(wx, wy):
                continue
            dist = math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2)
            if dist < self.goal_tolerance:
                continue
            scored.append((wx, wy, size, dist))

        # Uzaktan yakına sırala (en uzak ilk = index 0)
        scored.sort(key=lambda t: t[3], reverse=True)

        targets = [(s[0], s[1], s[2]) for s in scored]

        node = ExploreNode(position, targets)

        if targets:
            self.get_logger().info(
                f"Yeni DFS node: ({rx:.1f}, {ry:.1f}) | "
                f"{len(targets)} hedef | "
                f"en uzak: {scored[0][3]:.1f}m | "
                f"en yakin: {scored[-1][3]:.1f}m")

        return node

    def _is_blacklisted(self, wx, wy):
        for bx, by in self.blacklisted:
            if math.sqrt((wx - bx) ** 2 + (wy - by) ** 2) < self.blacklist_radius:
                return True
        return False

    # ════════════════════════════════════════════
    #  Goal Gönderme
    # ════════════════════════════════════════════

    def _send_goal(self, wx, wy):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(wx)
        msg.pose.position.y = float(wy)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)
        self.current_goal = (wx, wy)
        self.goal_sent_time = self.get_clock().now()

    # ════════════════════════════════════════════
    #  Ana DFS Keşif Döngüsü
    # ════════════════════════════════════════════

    def _explore_tick(self):
        if self.map_data is None:
            return

        robot_pos = self._get_robot_position()
        if robot_pos is None:
            self.get_logger().warn("Robot pozisyonu alinamadi (TF bekleniyor)")
            return

        # Durum yayınla
        status_msg = Bool()
        status_msg.data = self.exploring
        self.exploring_pub.publish(status_msg)

        # ── Aktif hedef varsa kontrol et ──
        if self.current_goal is not None:
            gx, gy = self.current_goal
            rx, ry = robot_pos
            dist = math.sqrt((gx - rx) ** 2 + (gy - ry) ** 2)

            if dist < self.goal_tolerance:
                # Hedefe ulaşıldı
                action = "BACKTRACK" if self.backtracking else "ILERLE"
                self.get_logger().info(
                    f"[{action}] Hedefe ulasildi ({gx:.1f}, {gy:.1f})")
                self.current_goal = None
                self.backtracking = False
                self.no_frontier_count = 0
                # Devam et — aşağıda yeni hedef seçilecek

            elif (self.goal_sent_time is not None and
                  (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
                  > self.goal_timeout):
                # Timeout — blacklist ve mevcut node'daki sonraki hedefe geç
                self.get_logger().warn(
                    f"Hedef timeout! Blacklist: ({gx:.1f}, {gy:.1f})")
                self.blacklisted.append((gx, gy))
                self.current_goal = None
                self.backtracking = False

                # Mevcut node varsa sonraki hedefe geç
                if self.explore_stack:
                    self.explore_stack[-1].advance()
            else:
                return  # Hala hedefe gidiyoruz

        # ── DFS mantığı ──
        frontiers = self._find_frontiers()

        if not frontiers:
            self.no_frontier_count += 1
            self.get_logger().info(
                f"Frontier bulunamadi ({self.no_frontier_count}/"
                f"{self.max_no_frontier})")
            if self.no_frontier_count >= self.max_no_frontier and self.exploring:
                self.get_logger().info(
                    "═══ KESIF TAMAMLANDI ═══ Tum alanlar kesfedildi!")
                self.exploring = False
                self.explore_stack.clear()
            return

        self.no_frontier_count = 0
        self.exploring = True

        # ── ADIM 1: Eğer stack boşsa veya mevcut dalda hedefe yeni ulaştıysak,
        #             yeni node oluştur ──
        if not self.explore_stack or not self.backtracking:
            node = self._create_explore_node(robot_pos, frontiers)

            if node.has_remaining:
                self.explore_stack.append(node)
            else:
                # Bu konumdan gidilecek yer yok → backtrack
                self._do_backtrack()
                return

        # ── ADIM 2: Stack'in tepesindeki node'dan sonraki hedefe git ──
        while self.explore_stack:
            top = self.explore_stack[-1]

            if top.has_remaining:
                target = top.current_target
                top.advance()  # Bir sonraki dal için index ilerlet

                rx, ry = robot_pos
                dist = math.sqrt(
                    (target[0] - rx) ** 2 + (target[1] - ry) ** 2)

                depth = len(self.explore_stack)
                remaining = len(top.targets) - top.target_idx
                self.get_logger().info(
                    f"[DFS derinlik={depth}] Hedef: ({target[0]:.1f}, "
                    f"{target[1]:.1f}) | mesafe: {dist:.1f}m | "
                    f"bu node'da kalan: {remaining}")

                self._send_goal(target[0], target[1])
                return
            else:
                # Bu node tükendi → pop ve backtrack
                self.explore_stack.pop()
                self.get_logger().info(
                    f"Dal tukendi, backtrack | stack derinlik: "
                    f"{len(self.explore_stack)}")

        # Stack tamamen boş — tüm dallar tükendi
        # Yeni frontier'lar doğmuş olabilir (harita güncellendi)
        node = self._create_explore_node(robot_pos, frontiers)
        if node.has_remaining:
            self.explore_stack.append(node)
            target = node.current_target
            node.advance()
            self.get_logger().info(
                f"[DFS yeniden baslat] Hedef: ({target[0]:.1f}, "
                f"{target[1]:.1f})")
            self._send_goal(target[0], target[1])
        else:
            self.get_logger().info(
                "Tum frontier'lar blacklist/yakin — bekleniyor")
            self.no_frontier_count += 1

    def _do_backtrack(self):
        """Stack'teki bir önceki node'a geri dön."""
        while self.explore_stack:
            top = self.explore_stack[-1]
            if top.has_remaining:
                # Bu node'da hala keşfedilecek dal var → pozisyonuna git
                self.backtracking = True
                bx, by = top.position
                self.get_logger().info(
                    f"[BACKTRACK] Geri donus: ({bx:.1f}, {by:.1f}) | "
                    f"stack derinlik: {len(self.explore_stack)}")
                self._send_goal(bx, by)
                return
            else:
                self.explore_stack.pop()

        self.get_logger().info("Stack bos — tum dallar kesfedildi")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Frontier Explorer durduruluyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
