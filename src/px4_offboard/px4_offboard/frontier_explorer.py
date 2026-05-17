"""
3D Frontier Explorer — Voxel-based point cloud exploration
==========================================================

3D nokta bulutu haritası üzerinden frontier keşfi yapar.
2D OccupancyGrid bağımlılığı tamamen kaldırılmıştır.

RTAB-Map'in /rtabmap/cloud_map (PointCloud2) topic'ini kullanır.
Nokta bulutunu voxel grid'e dönüştürüp 3D frontier tespiti yapar.
"""

import math
import struct
from collections import deque

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String


# Voxel states
UNKNOWN = 0
FREE = 1
OCCUPIED = 2


class VoxelGrid:
    """Lightweight 3D voxel grid built incrementally from live LiDAR scans."""

    def __init__(self, voxel_size=0.2, range_max=15.0):
        self.voxel_size = voxel_size
        self.range_max = range_max
        self.voxels = {}  # (ix, iy, iz) -> state

    def insert_scan(self, points_map, sensor_pos_map):
        """Insert a 3D scan (in map frame) into the voxel grid."""
        rx, ry, rz = sensor_pos_map
        sensor_voxel = self._to_voxel(rx, ry, rz)

        # Process every Nth point to save CPU
        for i in range(0, len(points_map), 4):
            px, py, pz = points_map[i]
            if np.isnan(px) or np.isnan(py) or np.isnan(pz):
                continue
            
            dist = math.sqrt((px - rx)**2 + (py - ry)**2 + (pz - rz)**2)
            if dist > self.range_max:
                continue

            target_voxel = self._to_voxel(px, py, pz)
            self._raycast_free(sensor_voxel, target_voxel)
            self.voxels[target_voxel] = OCCUPIED

    def _to_voxel(self, x, y, z):
        return (
            int(math.floor(x / self.voxel_size)),
            int(math.floor(y / self.voxel_size)),
            int(math.floor(z / self.voxel_size)),
        )

    def to_world(self, ix, iy, iz):
        s = self.voxel_size
        return ((ix + 0.5) * s, (iy + 0.5) * s, (iz + 0.5) * s)

    def _raycast_free(self, start, end):
        """Mark voxels along ray as FREE (excluding endpoint which is OCCUPIED)."""
        sx, sy, sz = start
        ex, ey, ez = end
        dx, dy, dz = ex - sx, ey - sy, ez - sz
        steps = max(abs(dx), abs(dy), abs(dz), 1)
        if steps > 150:
            steps = 150  # Limit for performance
        for i in range(steps):
            t = i / steps
            vx = int(round(sx + dx * t))
            vy = int(round(sy + dy * t))
            vz = int(round(sz + dz * t))
            key = (vx, vy, vz)
            if key not in self.voxels:
                self.voxels[key] = FREE
            elif self.voxels[key] == UNKNOWN:
                self.voxels[key] = FREE

    def get(self, ix, iy, iz):
        return self.voxels.get((ix, iy, iz), UNKNOWN)

    def get_bounds(self):
        if not self.voxels:
            return (0, 0, 0), (0, 0, 0)
        keys = list(self.voxels.keys())
        xs = [k[0] for k in keys]
        ys = [k[1] for k in keys]
        zs = [k[2] for k in keys]
        return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def parse_pointcloud2(msg):
    """Parse PointCloud2 message to Nx3 numpy array (x, y, z)."""
    point_step = msg.point_step
    data = bytes(msg.data)
    n_points = msg.width * msg.height

    # Find xyz field offsets
    offsets = {}
    for field in msg.fields:
        if field.name in ('x', 'y', 'z'):
            offsets[field.name] = field.offset

    if len(offsets) < 3:
        return np.zeros((0, 3), dtype=np.float32)

    points = np.zeros((n_points, 3), dtype=np.float32)
    ox, oy, oz = offsets['x'], offsets['y'], offsets['z']

    for i in range(n_points):
        base = i * point_step
        if base + oz + 4 > len(data):
            break
        points[i, 0] = struct.unpack_from('f', data, base + ox)[0]
        points[i, 1] = struct.unpack_from('f', data, base + oy)[0]
        points[i, 2] = struct.unpack_from('f', data, base + oz)[0]

    return points


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        # Parameters
        self.declare_parameter("min_frontier_size", 5)
        self.declare_parameter("goal_tolerance", 0.4)
        self.declare_parameter("update_interval", 1.0)
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("lidar_topic", "/drone/lidar/points")
        self.declare_parameter("goal_timeout", 15.0)
        self.declare_parameter("approach_offset", 1.5)
        self.declare_parameter("min_goal_distance", 1.5)
        self.declare_parameter("max_goal_distance", 8.0)
        self.declare_parameter("voxel_size", 0.2)
        self.declare_parameter("flight_z_tolerance", 0.5)
        self.declare_parameter("approach_safety_radius", 0)
        self.declare_parameter("utility_size_weight", 1.0)
        self.declare_parameter("utility_distance_weight", 1.5)
        self.declare_parameter("utility_direction_weight", 1.5)
        self.declare_parameter("blacklist_radius", 1.0)
        self.declare_parameter("blacklist_ttl_sec", 30.0)
        self.declare_parameter("recent_goal_radius", 1.2)
        self.declare_parameter("recent_goal_memory", 10)
        self.declare_parameter("max_no_frontier_cycles", 8)

        self.min_frontier_size = int(self.get_parameter("min_frontier_size").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.update_interval = float(self.get_parameter("update_interval").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout").value)
        self.approach_offset = float(self.get_parameter("approach_offset").value)
        self.min_goal_distance = float(self.get_parameter("min_goal_distance").value)
        self.max_goal_distance = float(self.get_parameter("max_goal_distance").value)
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.flight_z_tolerance = float(self.get_parameter("flight_z_tolerance").value)
        self.approach_safety_radius = int(self.get_parameter("approach_safety_radius").value)
        self.utility_size_weight = float(self.get_parameter("utility_size_weight").value)
        self.utility_distance_weight = float(self.get_parameter("utility_distance_weight").value)
        self.utility_direction_weight = float(self.get_parameter("utility_direction_weight").value)
        self.blacklist_radius = float(self.get_parameter("blacklist_radius").value)
        self.blacklist_ttl_sec = float(self.get_parameter("blacklist_ttl_sec").value)
        self.recent_goal_radius = float(self.get_parameter("recent_goal_radius").value)
        self.recent_goal_memory = max(1, int(self.get_parameter("recent_goal_memory").value))
        self.max_no_frontier_cycles = max(1, int(self.get_parameter("max_no_frontier_cycles").value))

        # Voxel grid
        self.voxel_grid = VoxelGrid(self.voxel_size)

        # Subscribers
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PointCloud2, self.lidar_topic, self._lidar_cb, lidar_qos)
        self.create_subscription(String, "/navigator/status", self._nav_status_cb, 10)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.exploring_pub = self.create_publisher(Bool, "/explorer/active", 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.last_scan_time = 0.0
        self.current_goal = None
        self.goal_sent_time = None
        self.blacklisted = []
        self.recent_goals = deque(maxlen=self.recent_goal_memory)
        self.no_frontier_count = 0
        self.exploring = False

        # Timer
        self.timer = self.create_timer(self.update_interval, self._explore_tick)

        self.get_logger().info(
            f"3D Frontier Explorer started | lidar={self.lidar_topic} | "
            f"voxel={self.voxel_size}m")

    # ── Helpers ──

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _prune_blacklist(self):
        if not self.blacklisted:
            return
        now = self._now_sec()
        self.blacklisted = [(x, y, z, t) for x, y, z, t in self.blacklisted if t > now]

    def _add_blacklist(self, wx, wy, wz):
        self.blacklisted.append((wx, wy, wz, self._now_sec() + self.blacklist_ttl_sec))

    def _is_blacklisted(self, wx, wy, wz):
        self._prune_blacklist()
        for bx, by, bz, _ in self.blacklisted:
            if math.sqrt((wx-bx)**2 + (wy-by)**2 + (wz-bz)**2) < self.blacklist_radius:
                return True
        return False

    def _is_recent_goal(self, wx, wy):
        for rx, ry, _ in self.recent_goals:
            if math.hypot(wx - rx, wy - ry) < self.recent_goal_radius:
                return True
        return False

    # ── Callbacks ──

    def _lidar_cb(self, msg: PointCloud2):
        """Process live LiDAR scan and insert into voxel grid."""
        now = self._now_sec()
        if now - self.last_scan_time < 0.5:  # Process max 2 scans per second
            return
        self.last_scan_time = now

        # 1. Parse point cloud (in lidar_link frame)
        pts_lidar = parse_pointcloud2(msg)
        if len(pts_lidar) == 0:
            return

        # 2. Get transform lidar_link -> map
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tz = tf.transform.translation.z
            q = tf.transform.rotation
            
            # Simple quaternion to rotation matrix applied to points
            # Formula: v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
            # Using numpy for fast vectorization
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            
            # Precompute rotation matrix
            R = np.array([
                [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
                [    2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
                [    2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
            ])
            
            # Transform all points: R * p + t
            pts_map = np.dot(pts_lidar, R.T) + np.array([tx, ty, tz])
            
            # Insert into voxel grid
            self.voxel_grid.insert_scan(pts_map, (tx, ty, tz))
            
        except Exception as e:
            self.get_logger().warn(f"TF error in lidar_cb: {e}")

    def _nav_status_cb(self, msg: String):
        if self.current_goal is None:
            return
        gx, gy, gz = self.current_goal
        status = msg.data
        if status == "FAILED":
            self.get_logger().warn(f"Nav failed -> blacklist ({self.current_goal[0]:.1f}, {self.current_goal[1]:.1f}, {self.current_goal[2]:.1f})")
            self.blacklisted.append(self.current_goal + (self.get_clock().now().nanoseconds / 1e9,))
            self.current_goal = None
        elif status == "REPLAN":
            self.get_logger().warn(f"Obstacle on path -> replan for ({self.current_goal[0]:.1f}, {self.current_goal[1]:.1f}, {self.current_goal[2]:.1f})")
            self.current_goal = None  # Blacklist'e eklemeden tekrar dene
            self.goal_sent_time = None
        elif status == "REACHED":
            self.get_logger().info(f"Reached ({gx:.1f}, {gy:.1f}, {gz:.1f})")
            self.recent_goals.append((gx, gy, gz))
            self.current_goal = None
            self.goal_sent_time = None
            self.no_frontier_count = 0

    def _get_robot_position(self):
        """Return (x, y, z, yaw) in map frame or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", self.robot_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.3))
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z
            q = tf.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            return (x, y, z, yaw)
        except Exception:
            return None

    # ── 3D Frontier Detection ──

    def _find_frontiers(self, robot_pos):
        """Find 3D frontier clusters in the voxel grid."""
        if not self.voxel_grid.voxels:
            return []

        rx, ry, rz = robot_pos[:3]
        vg = self.voxel_grid

        # Find frontier voxels: FREE voxels with at least one UNKNOWN neighbor
        frontier_voxels = []
        neighbors_6 = [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]

        for key, state in vg.voxels.items():
            if state != FREE:
                continue
            ix, iy, iz = key
            for dx, dy, dz in neighbors_6:
                nk = (ix+dx, iy+dy, iz+dz)
                if vg.get(*nk) == UNKNOWN:
                    frontier_voxels.append(key)
                    break

        if not frontier_voxels:
            return []

        # Cluster frontiers using BFS
        visited = set()
        frontiers = []
        fset = set(frontier_voxels)

        for voxel in frontier_voxels:
            if voxel in visited:
                continue
            queue = deque([voxel])
            visited.add(voxel)
            cluster = []
            while queue:
                v = queue.popleft()
                cluster.append(v)
                ix, iy, iz = v
                for dx, dy, dz in neighbors_6:
                    nk = (ix+dx, iy+dy, iz+dz)
                    if nk not in visited and nk in fset:
                        visited.add(nk)
                        queue.append(nk)

            if len(cluster) < self.min_frontier_size:
                continue

            # Centroid in world coordinates
            cx = sum(v[0] for v in cluster) / len(cluster)
            cy = sum(v[1] for v in cluster) / len(cluster)
            cz = sum(v[2] for v in cluster) / len(cluster)
            wx, wy, wz = vg.to_world(int(round(cx)), int(round(cy)), int(round(cz)))
            frontiers.append((wx, wy, wz, len(cluster)))

        return frontiers

    def _is_approach_safe(self, wx, wy, wz):
        """Check if approach point is in safe free space."""
        vg = self.voxel_grid
        ix, iy, iz = vg._to_voxel(wx, wy, wz)

        # Check immediate area for obstacles
        r = self.approach_safety_radius
        occupied_count = 0
        free_count = 0
        total = 0
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                for dz in range(-1, 2):  # +-1 in z
                    total += 1
                    state = vg.get(ix+dx, iy+dy, iz+dz)
                    if state == OCCUPIED:
                        occupied_count += 1
                    elif state == FREE:
                        free_count += 1

        if occupied_count > 0:
            return False
        if total > 0 and free_count / total < 0.5:
            return False
        return True

    def _compute_approach_goal(self, fx, fy, fz, rx, ry, rz):
        dx, dy = rx - fx, ry - fy
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return fx, fy, fz
        offset = min(self.approach_offset, dist * 0.35)
        gx = fx + (dx / dist) * offset
        gy = fy + (dy / dist) * offset
        return gx, gy, fz  # Keep frontier z for 3D support

    def _select_goal(self, frontiers, robot_pos):
        rx, ry, rz, yaw = robot_pos
        if not frontiers:
            return None

        max_size = max(s for _, _, _, s in frontiers)
        candidates = []
        
        reasons = {"dist": 0, "blacklist": 0, "memory": 0, "safety": 0}

        for fx, fy, fz, size in frontiers:
            dist = math.sqrt((fx-rx)**2 + (fy-ry)**2 + (fz-rz)**2)
            if dist < self.min_goal_distance or dist > self.max_goal_distance:
                reasons["dist"] += 1
                continue

            ax, ay, az = self._compute_approach_goal(fx, fy, fz, rx, ry, rz)

            if self._is_blacklisted(ax, ay, az) or self._is_blacklisted(fx, fy, fz):
                reasons["blacklist"] += 1
                continue
            if self._is_recent_goal(ax, ay):
                reasons["memory"] += 1
                continue

            if not self._is_approach_safe(ax, ay, az):
                reasons["safety"] += 1
                continue

            size_score = size / max_size if max_size > 0 else 0.0
            dist_score = 1.0 / (1.0 + dist)

            angle_to = math.atan2(fy - ry, fx - rx)
            angle_diff = abs(math.atan2(math.sin(angle_to - yaw), math.cos(angle_to - yaw)))
            dir_score = 1.0 - (angle_diff / math.pi)

            utility = (self.utility_size_weight * size_score
                      + self.utility_distance_weight * dist_score
                      + self.utility_direction_weight * dir_score)

            candidates.append((utility, dist, fx, fy, fz, size, ax, ay, az))

        if not candidates:
            self.get_logger().info(
                f"No selectable frontier (filtered). Total={len(frontiers)} | "
                f"dist={reasons['dist']} blacklist={reasons['blacklist']} "
                f"memory={reasons['memory']} safety={reasons['safety']}")
            return None
        candidates.sort(key=lambda c: (-c[0], c[1]))
        return candidates[0]

    def _send_goal(self, wx, wy, wz):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(wx)
        msg.pose.position.y = float(wy)
        msg.pose.position.z = float(wz)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.current_goal = (wx, wy, wz)
        self.goal_sent_time = self.get_clock().now()

    def _explore_tick(self):
        if not self.voxel_grid.voxels:
            return

        robot_pos = self._get_robot_position()
        if robot_pos is None:
            self.get_logger().warn("Robot pose unavailable")
            return

        self._prune_blacklist()

        active_msg = Bool()
        active_msg.data = self.exploring
        self.exploring_pub.publish(active_msg)

        # Keep current goal alive
        if self.current_goal is not None and self.goal_sent_time is not None:
            gx, gy, gz = self.current_goal
            rx, ry, rz, _ = robot_pos
            dist = math.sqrt((gx-rx)**2 + (gy-ry)**2 + (gz-rz)**2)

            if dist < self.goal_tolerance:
                self.get_logger().info(f"Reached target ({gx:.1f}, {gy:.1f}, {gz:.1f})")
                self.recent_goals.append((gx, gy, gz))
                self.current_goal = None
                self.goal_sent_time = None
                self.no_frontier_count = 0
            else:
                elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
                if elapsed > self.goal_timeout:
                    self.get_logger().warn(f"Goal timeout -> blacklist ({gx:.1f}, {gy:.1f})")
                    self._add_blacklist(gx, gy, gz)
                    self.current_goal = None
                    self.goal_sent_time = None
                else:
                    self.exploring = True
                    return

        frontiers = self._find_frontiers(robot_pos)
        if not frontiers:
            self.no_frontier_count += 1
            if self.no_frontier_count % 2 == 0:
                self.get_logger().info(
                    f"No frontier ({self.no_frontier_count}/{self.max_no_frontier_cycles})")
            if self.no_frontier_count >= self.max_no_frontier_cycles and self.exploring:
                self.get_logger().info("Exploration completed")
                self.exploring = False
            return

        choice = self._select_goal(frontiers, robot_pos)
        if choice is None:
            self.no_frontier_count += 1
            self.get_logger().info("No selectable frontier (filtered)")
            if self.no_frontier_count >= 3 and len(self.recent_goals) > 0:
                self.recent_goals.clear()
                self.get_logger().info("Recent-goal memory cleared")
            return

        utility, dist, fx, fy, fz, size, ax, ay, az = choice
        self._send_goal(ax, ay, az)
        self.exploring = True
        self.no_frontier_count = 0

        self.get_logger().info(
            f"Goal | u={utility:.3f} d={dist:.1f}m | "
            f"frontier=({fx:.1f},{fy:.1f},{fz:.1f}) s={size} | "
            f"approach=({ax:.1f},{ay:.1f},{az:.1f})")


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
