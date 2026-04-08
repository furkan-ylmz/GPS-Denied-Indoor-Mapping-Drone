"""
Map Cleaner — OccupancyGrid post-processing
===========================================

/map topic'indeki occupancy grid'i temizleyip /map_clean olarak yayınlar.
Amaç:
  - Küçük gürültü/tekil engel kümelerini temizlemek
  - Serbest alan içindeki küçük unknown ceplerini yumuşatmak

Not: Ham /map korunur, sadece yeni bir temiz topic üretilir.
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid


MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class MapCleaner(Node):
    def __init__(self):
        super().__init__("map_cleaner")

        self.declare_parameter("input_topic", "/map")
        self.declare_parameter("output_topic", "/map_clean")
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("min_obstacle_cluster_size", 10)
        self.declare_parameter("enable_unknown_hole_fill", True)
        self.declare_parameter("unknown_free_neighbor_threshold", 6)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.min_obstacle_cluster_size = int(
            self.get_parameter("min_obstacle_cluster_size").value
        )
        self.enable_unknown_hole_fill = bool(
            self.get_parameter("enable_unknown_hole_fill").value
        )
        self.unknown_free_neighbor_threshold = int(
            self.get_parameter("unknown_free_neighbor_threshold").value
        )

        self.create_subscription(
            OccupancyGrid,
            input_topic,
            self._map_cb,
            MAP_QOS,
        )
        self.pub = self.create_publisher(OccupancyGrid, output_topic, MAP_QOS)

        self._msg_count = 0
        self.get_logger().info(
            "MapCleaner baslatildi | "
            f"{input_topic} -> {output_topic} | "
            f"min_cluster={self.min_obstacle_cluster_size}"
        )

    def _remove_small_obstacle_clusters(self, grid: np.ndarray) -> int:
        occupied = grid >= self.occupied_threshold
        h, w = occupied.shape
        visited = np.zeros_like(occupied, dtype=bool)
        removed_cells = 0

        # 8-komsuluk ile bagli komponent analizi
        neighbors = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1),
        ]

        occ_indices = np.argwhere(occupied)
        for r, c in occ_indices:
            if visited[r, c]:
                continue

            queue = deque([(int(r), int(c))])
            visited[r, c] = True
            cluster = []

            while queue:
                rr, cc = queue.popleft()
                cluster.append((rr, cc))

                for dr, dc in neighbors:
                    nr = rr + dr
                    nc = cc + dc
                    if nr < 0 or nr >= h or nc < 0 or nc >= w:
                        continue
                    if visited[nr, nc] or not occupied[nr, nc]:
                        continue
                    visited[nr, nc] = True
                    queue.append((nr, nc))

            if len(cluster) < self.min_obstacle_cluster_size:
                for rr, cc in cluster:
                    grid[rr, cc] = 0
                removed_cells += len(cluster)

        return removed_cells

    def _fill_unknown_holes(self, grid: np.ndarray) -> int:
        if not self.enable_unknown_hole_fill:
            return 0

        unknown_mask = grid == -1
        free_mask = grid == 0

        h, w = grid.shape
        padded_free = np.pad(free_mask, 1, mode="constant", constant_values=False)

        # 8-komsudaki free hucre sayisini hesapla (wrap-around olmadan)
        free_neighbor_count = np.zeros((h, w), dtype=np.int16)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                free_neighbor_count += padded_free[
                    1 + dr:1 + dr + h,
                    1 + dc:1 + dc + w,
                ]

        fill_mask = unknown_mask & (
            free_neighbor_count >= self.unknown_free_neighbor_threshold
        )
        filled = int(np.count_nonzero(fill_mask))
        if filled > 0:
            grid[fill_mask] = 0
        return filled

    def _map_cb(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        if w == 0 or h == 0:
            return

        raw = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
        cleaned = raw.copy()

        removed_obstacles = self._remove_small_obstacle_clusters(cleaned)
        filled_unknown = self._fill_unknown_holes(cleaned)

        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = cleaned.astype(np.int8).reshape(-1).tolist()
        self.pub.publish(out)

        self._msg_count += 1
        if self._msg_count % 20 == 0:
            self.get_logger().info(
                "map temizlendi | "
                f"kaldirilan_kucuk_engel={removed_obstacles} | "
                f"doldurulan_unknown={filled_unknown}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MapCleaner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
