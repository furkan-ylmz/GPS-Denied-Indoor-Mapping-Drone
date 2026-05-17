"""
Frontier Explorer - Utility-based WFD exploration
=================================================

This node explores unknown areas by selecting frontiers from a cleaned map
using a utility score.

Design goals:
- Avoid repetitive DFS backtracking loops.
- Prefer frontiers with higher information gain while keeping travel efficient.
- Reduce re-targeting recently attempted or failed goals.
"""

import math
from collections import deque

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String


UNKNOWN = -1
FREE = 0


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        # Core behavior
        self.declare_parameter("min_frontier_size", 8)
        self.declare_parameter("goal_tolerance", 0.8)
        self.declare_parameter("update_interval", 0.5)
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("costmap_topic", "/map_clean")
        self.declare_parameter("goal_timeout", 15.0)
        self.declare_parameter("approach_offset", 3.5)       # 2.5 → 3.5: frontier'dan daha uzak dur
        self.declare_parameter("min_goal_distance", 1.5)     # 1.1 → 1.5: çok yakın hedefleri reddet

        # Approach safety filter — duvara yakın hedefleri engelle
        self.declare_parameter("approach_safety_radius_cells", 5)  # 1 → 5: 0.5m güvenlik çapı
        self.declare_parameter("approach_occupied_threshold", 65)
        self.declare_parameter("approach_min_free_ratio", 0.70)    # 0.6 → 0.70: daha fazla boş alan şart

        # Utility score tuning
        self.declare_parameter("utility_size_weight", 1.0)
        self.declare_parameter("utility_distance_weight", 1.2)
        self.declare_parameter("utility_direction_weight", 0.8)  # Yön bonusu

        # Anti-repeat / stuck handling
        self.declare_parameter("blacklist_radius", 1.0)
        self.declare_parameter("blacklist_ttl_sec", 30.0)
        self.declare_parameter("recent_goal_radius", 1.2)
        self.declare_parameter("recent_goal_memory", 10)
        self.declare_parameter("max_no_frontier_cycles", 8)

        self.min_frontier_size = int(self.get_parameter("min_frontier_size").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.update_interval = float(self.get_parameter("update_interval").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.costmap_topic = str(self.get_parameter("costmap_topic").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout").value)
        self.approach_offset = float(self.get_parameter("approach_offset").value)
        self.min_goal_distance = float(self.get_parameter("min_goal_distance").value)
        self.approach_safety_radius_cells = max(
            0, int(self.get_parameter("approach_safety_radius_cells").value)
        )
        self.approach_occupied_threshold = int(
            self.get_parameter("approach_occupied_threshold").value
        )
        self.approach_min_free_ratio = float(
            self.get_parameter("approach_min_free_ratio").value
        )

        self.utility_size_weight = float(
            self.get_parameter("utility_size_weight").value
        )
        self.utility_distance_weight = float(
            self.get_parameter("utility_distance_weight").value
        )
        self.utility_direction_weight = float(
            self.get_parameter("utility_direction_weight").value
        )

        self.blacklist_radius = float(self.get_parameter("blacklist_radius").value)
        self.blacklist_ttl_sec = float(self.get_parameter("blacklist_ttl_sec").value)
        self.recent_goal_radius = float(self.get_parameter("recent_goal_radius").value)
        self.recent_goal_memory = max(
            1, int(self.get_parameter("recent_goal_memory").value)
        )
        self.max_no_frontier_cycles = max(
            1, int(self.get_parameter("max_no_frontier_cycles").value)
        )

        # Subscribers
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self._map_cb,
            map_qos,
        )
        self.create_subscription(String, "/navigator/status", self._nav_status_cb, 10)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.exploring_pub = self.create_publisher(Bool, "/explorer/active", 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.map_data = None
        self.map_info = None

        self.current_goal = None
        self.goal_sent_time = None

        self.blacklisted = []
        self.recent_goals = deque(maxlen=self.recent_goal_memory)

        self.no_frontier_count = 0
        self.exploring = False

        # Timer
        self.timer = self.create_timer(self.update_interval, self._explore_tick)

        self.get_logger().info(
            "Utility WFD explorer started | "
            f"map={self.costmap_topic} | "
            f"weights(size/dist/dir)=({self.utility_size_weight:.2f}/"
            f"{self.utility_distance_weight:.2f}/"
            f"{self.utility_direction_weight:.2f})"
        )

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _prune_blacklist(self):
        if not self.blacklisted:
            return
        now_sec = self._now_sec()
        self.blacklisted = [
            (bx, by, expires_at)
            for bx, by, expires_at in self.blacklisted
            if expires_at > now_sec
        ]

    def _add_blacklist(self, wx, wy):
        expires_at = self._now_sec() + self.blacklist_ttl_sec
        self.blacklisted.append((wx, wy, expires_at))

    def _map_cb(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.map_data = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )

    def _nav_status_cb(self, msg: String):
        if self.current_goal is None:
            return

        gx, gy = self.current_goal
        if msg.data == "FAILED":
            self.get_logger().warn(
                f"Navigator failed -> temporary blacklist ({gx:.1f}, {gy:.1f})"
            )
            self._add_blacklist(gx, gy)
            self.current_goal = None
            self.goal_sent_time = None
        elif msg.data == "REACHED":
            self.get_logger().info(
                f"Navigator reached goal ({gx:.1f}, {gy:.1f})"
            )
            self.recent_goals.append((gx, gy))
            self.current_goal = None
            self.goal_sent_time = None
            self.no_frontier_count = 0

    def _get_robot_position(self):
        """Robot pozisyonunu ve yaw açısını döndürür: (x, y, yaw) veya None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.3),
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            # Quaternion -> yaw
            q = tf.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
        except Exception:
            return None

    def _grid_to_world(self, gx, gy):
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        return (ox + (gx + 0.5) * res, oy + (gy + 0.5) * res)

    def _world_to_grid(self, wx, wy):
        if self.map_info is None:
            return None

        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution

        if res <= 0.0:
            return None

        col = int((wx - ox) / res)
        row = int((wy - oy) / res)

        if row < 0 or row >= self.map_info.height or col < 0 or col >= self.map_info.width:
            return None

        return row, col

    def _is_blacklisted(self, wx, wy):
        self._prune_blacklist()
        for bx, by, _ in self.blacklisted:
            if math.hypot(wx - bx, wy - by) < self.blacklist_radius:
                return True
        return False

    def _is_recent_goal(self, wx, wy):
        for rx, ry in self.recent_goals:
            if math.hypot(wx - rx, wy - ry) < self.recent_goal_radius:
                return True
        return False

    def _is_approach_safe(self, wx, wy):
        """Hedef noktasının güvenli olup olmadığını kontrol et.
        
        Engel yokluğu, yeterli boş alan ve düşük unknown oranı gerektirir.
        Keşfedilmemiş alanların ortasına hedef göndermeyi engeller.
        """
        if self.map_data is None or self.map_info is None:
            return False

        rc = self._world_to_grid(wx, wy)
        if rc is None:
            return False

        row, col = rc
        if self.map_data[row, col] != FREE:
            return False

        h, w = self.map_data.shape
        radius = self.approach_safety_radius_cells
        r0 = max(0, row - radius)
        r1 = min(h, row + radius + 1)
        c0 = max(0, col - radius)
        c1 = min(w, col + radius + 1)
        patch = self.map_data[r0:r1, c0:c1]

        occupied_count = int(
            np.count_nonzero(patch >= self.approach_occupied_threshold)
        )
        if occupied_count > 0:
            return False

        total = float(patch.size)
        free_ratio = float(np.count_nonzero(patch == FREE)) / total
        if free_ratio < self.approach_min_free_ratio:
            return False

        # Keşfedilmemiş alan oranı kontrolü — %40'tan fazla unknown varsa tehlikeli
        unknown_ratio = float(np.count_nonzero(patch == UNKNOWN)) / total
        if unknown_ratio > 0.40:
            return False

        return True

    def _find_frontiers(self):
        if self.map_data is None or self.map_info is None:
            return []

        h, w = self.map_data.shape
        free_mask = self.map_data == FREE
        unknown_mask = self.map_data == UNKNOWN

        # A frontier cell is free and has at least one unknown 4-neighbor.
        has_unknown_neighbor = np.zeros((h, w), dtype=bool)
        if h > 1:
            has_unknown_neighbor[1:, :] |= unknown_mask[:-1, :]
            has_unknown_neighbor[:-1, :] |= unknown_mask[1:, :]
        if w > 1:
            has_unknown_neighbor[:, 1:] |= unknown_mask[:, :-1]
            has_unknown_neighbor[:, :-1] |= unknown_mask[:, 1:]

        frontier_mask = free_mask & has_unknown_neighbor
        frontier_coords = np.argwhere(frontier_mask)
        if len(frontier_coords) == 0:
            return []

        visited = set()
        frontiers = []

        for row, col in frontier_coords:
            row = int(row)
            col = int(col)
            if (row, col) in visited:
                continue

            queue = deque([(row, col)])
            visited.add((row, col))
            cluster_cells = []

            while queue:
                r, c = queue.popleft()
                cluster_cells.append((r, c))

                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr = r + dr
                    nc = c + dc
                    if nr < 0 or nr >= h or nc < 0 or nc >= w:
                        continue
                    if (nr, nc) in visited:
                        continue
                    if not frontier_mask[nr, nc]:
                        continue
                    visited.add((nr, nc))
                    queue.append((nr, nc))

            size = len(cluster_cells)
            if size < self.min_frontier_size:
                continue

            cx = sum(c for _, c in cluster_cells) / size
            cy = sum(r for r, _ in cluster_cells) / size
            wx, wy = self._grid_to_world(cx, cy)
            frontiers.append((wx, wy, size))

        return frontiers

    def _compute_approach_goal(self, fx, fy, rx, ry):
        dx = rx - fx
        dy = ry - fy
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return fx, fy

        offset = min(self.approach_offset, dist * 0.35)
        gx = fx + (dx / dist) * offset
        gy = fy + (dy / dist) * offset
        return gx, gy

    def _select_goal(self, frontiers, robot_pos):
        rx, ry, robot_yaw = robot_pos
        if not frontiers:
            return None

        max_size = max(size for _, _, size in frontiers)
        candidates = []

        for fx, fy, size in frontiers:
            dist = math.hypot(fx - rx, fy - ry)
            if dist < self.min_goal_distance:
                continue

            ax, ay = self._compute_approach_goal(fx, fy, rx, ry)
            
            if self._is_blacklisted(ax, ay) or self._is_blacklisted(fx, fy):
                continue
            if self._is_recent_goal(ax, ay) or self._is_recent_goal(fx, fy):
                continue

            size_score = size / max_size if max_size > 0 else 0.0
            distance_score = 1.0 / (1.0 + dist)

            # Yön bonusu: drone'un baktığı yöne yakın frontier'lara puan ver
            # Böylece gereksiz 180° dönüşler azalır
            angle_to_frontier = math.atan2(fy - ry, fx - rx)
            angle_diff = abs(math.atan2(
                math.sin(angle_to_frontier - robot_yaw),
                math.cos(angle_to_frontier - robot_yaw)
            ))
            # 0 derece fark = 1.0 puan, 180 derece fark = 0.0 puan
            direction_score = 1.0 - (angle_diff / math.pi)

            utility = (
                self.utility_size_weight * size_score
                + self.utility_distance_weight * distance_score
                + self.utility_direction_weight * direction_score
            )

            if not self._is_approach_safe(ax, ay):
                continue
            candidates.append((utility, dist, fx, fy, size, ax, ay))

        if not candidates:
            return None

        # Highest utility first, then nearest among equals.
        candidates.sort(key=lambda c: (-c[0], c[1]))
        return candidates[0]

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

    def _explore_tick(self):
        if self.map_data is None:
            return

        robot_pos = self._get_robot_position()
        if robot_pos is None:
            self.get_logger().warn("Robot pose unavailable, waiting for TF")
            return

        self._prune_blacklist()

        active_msg = Bool()
        active_msg.data = self.exploring
        self.exploring_pub.publish(active_msg)

        # Keep current goal alive until reached or timeout.
        if self.current_goal is not None and self.goal_sent_time is not None:
            gx, gy = self.current_goal
            rx, ry, _ = robot_pos
            dist = math.hypot(gx - rx, gy - ry)

            if dist < self.goal_tolerance:
                self.get_logger().info(
                    f"Explorer reached target ({gx:.1f}, {gy:.1f})"
                )
                self.recent_goals.append((gx, gy))
                self.current_goal = None
                self.goal_sent_time = None
                self.no_frontier_count = 0
            else:
                elapsed = (
                    self.get_clock().now() - self.goal_sent_time
                ).nanoseconds / 1e9
                if elapsed > self.goal_timeout:
                    self.get_logger().warn(
                        f"Goal timeout -> temporary blacklist ({gx:.1f}, {gy:.1f})"
                    )
                    self._add_blacklist(gx, gy)
                    self.current_goal = None
                    self.goal_sent_time = None
                else:
                    self.exploring = True
                    return

        frontiers = self._find_frontiers()
        if not frontiers:
            self.no_frontier_count += 1
            if self.no_frontier_count % 2 == 0:
                self.get_logger().info(
                    f"No frontier detected ({self.no_frontier_count}/"
                    f"{self.max_no_frontier_cycles})"
                )
            if self.no_frontier_count >= self.max_no_frontier_cycles and self.exploring:
                self.get_logger().info("Exploration completed: no frontier remains")
                self.exploring = False
            return

        choice = self._select_goal(frontiers, robot_pos)
        if choice is None:
            self.no_frontier_count += 1
            self.get_logger().info(
                "No selectable frontier (blacklist/recent/safety filters active)"
            )
            # Prevent deadlock when all candidates are filtered.
            if self.no_frontier_count >= 3 and len(self.recent_goals) > 0:
                self.recent_goals.clear()
                self.get_logger().info("Recent-goal memory cleared to continue exploration")
            return

        utility, dist, fx, fy, size, ax, ay = choice
        self._send_goal(ax, ay)
        self.exploring = True
        self.no_frontier_count = 0

        self.get_logger().info(
            f"Goal selected | utility={utility:.3f} | dist={dist:.1f}m | "
            f"frontier=({fx:.1f}, {fy:.1f}) size={size} | "
            f"approach=({ax:.1f}, {ay:.1f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Frontier explorer stopped")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
