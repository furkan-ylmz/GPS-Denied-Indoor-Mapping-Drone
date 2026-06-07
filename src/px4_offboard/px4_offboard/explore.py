"""
Frontier Explorer — Otonom Keşif Düğümü

RTAB-Map'ten gelen OccupancyGrid haritasını analiz ederek frontier (keşfedilmemiş sınır)
noktalarını tespit eder, kümeleme tabanlı strateji ile en uygun hedefi seçer ve
Nav2'ye NavigateToPose hedefi göndererek dronu otonom keşfe yönlendirir.

Durum Makinesi:
  WAITING_FOR_MAP → INITIALIZING → EXPLORING → RETURNING_HOME → COMPLETED

Kümeleme Stratejisi:
  - Yakın kümeler varsa (< nearby_threshold) önce yerel alanı bitir
  - Yakın küme yoksa skor = boyut / mesafe ile en verimli kümeyi seç
  - Ping-pong engellenir: dron bir bölgeyi bitirmeden uzağa gitmez
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

# Durum makinesi durumları
STATE_WAITING_FOR_MAP = 0
STATE_INITIALIZING = 1
STATE_EXPLORING = 2
STATE_RETURNING_HOME = 3
STATE_COMPLETED = 4

# OccupancyGrid hücre değerleri
CELL_FREE = 0
CELL_UNKNOWN = -1
# Engel = 1-100 arası (genellikle 100)

# 8-komşuluk yönleri
NEIGHBORS_8 = [(-1, -1), (-1, 0), (-1, 1),
               (0, -1),           (0, 1),
               (1, -1),  (1, 0),  (1, 1)]


class FrontierExplorer(Node):
    """Frontier tabanlı otonom keşif düğümü."""

    def __init__(self):
        super().__init__('frontier_explorer')

        # --- ROS2 Parametreleri ---
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('nearby_threshold', 2.0)
        self.declare_parameter('min_goal_distance', 0.8)
        self.declare_parameter('map_update_interval', 1.0)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('blacklist_radius', 1.0)
        self.declare_parameter('home_tolerance', 0.5)
        self.declare_parameter('init_wait_time', 10.0)

        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.nearby_threshold = self.get_parameter('nearby_threshold').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.map_update_interval = self.get_parameter('map_update_interval').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.home_tolerance = self.get_parameter('home_tolerance').value
        self.init_wait_time = self.get_parameter('init_wait_time').value

        # --- Durum Değişkenleri ---
        self.state = STATE_WAITING_FOR_MAP
        self.home_position = None       # (x, y) başlangıç konumu
        self.current_map = None         # Son alınan OccupancyGrid
        self.blacklisted_goals = []     # Başarısız hedefler listesi
        self.init_start_time = None     # Bekleme başlangıç zamanı
        self.goal_active = False        # Nav2 hedefi aktif mi?
        self.goal_handle = None         # Mevcut Nav2 goal handle
        self.current_target = None      # Mevcut hedef (x, y)
        self.current_target_cluster = None  # Görselleştirme için seçilen küme

        # --- TF2 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Subscriber'lar ---
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        # --- Publisher'lar ---
        self.command_pub = self.create_publisher(String, '/explore/command', 10)
        self.frontiers_pub = self.create_publisher(
            MarkerArray, '/explore/frontiers', 10)
        self.target_pub = self.create_publisher(
            MarkerArray, '/explore/target', 10)

        # --- Nav2 Action Client ---
        self.nav2_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # --- Zamanlayıcılar ---
        self.explore_timer = self.create_timer(
            self.map_update_interval, self._explore_tick)

        self.get_logger().info('🗺️  Frontier Explorer başlatıldı')
        self.get_logger().info(f'  min_frontier_size: {self.min_frontier_size}')
        self.get_logger().info(f'  nearby_threshold: {self.nearby_threshold}m')
        self.get_logger().info(f'  min_goal_distance: {self.min_goal_distance}m')
        self.get_logger().info(f'  init_wait_time: {self.init_wait_time}s')
        self.get_logger().info(f'  goal_timeout: {self.goal_timeout}s')

    # ─────────────────────────────────────────────────────────
    # Callback'ler
    # ─────────────────────────────────────────────────────────

    def _map_cb(self, msg):
        """RTAB-Map'ten gelen harita güncellemesi."""
        self.current_map = msg

        if self.state == STATE_WAITING_FOR_MAP:
            self.get_logger().info('📡 İlk harita alındı — Başlatma aşamasına geçiliyor')
            self.state = STATE_INITIALIZING
            self.init_start_time = self.get_clock().now()

    # ─────────────────────────────────────────────────────────
    # Ana Keşif Döngüsü
    # ─────────────────────────────────────────────────────────

    def _explore_tick(self):
        """Periyodik keşif durumu kontrolü."""

        if self.state == STATE_WAITING_FOR_MAP:
            return

        elif self.state == STATE_INITIALIZING:
            self._handle_initializing()

        elif self.state == STATE_EXPLORING:
            self._handle_exploring()

        elif self.state == STATE_RETURNING_HOME:
            self._handle_returning_home()

        elif self.state == STATE_COMPLETED:
            pass  # İniş sinyali gönderildi, iş bitti

    def _handle_initializing(self):
        """Kalkış sonrası bekleme — RTAB-Map'in haritayı oluşturması için."""
        elapsed = (self.get_clock().now() - self.init_start_time).nanoseconds / 1e9
        remaining = self.init_wait_time - elapsed

        if remaining > 0:
            self.get_logger().info(
                f'⏳ Başlatma bekleniyor... {remaining:.0f}s kaldı',
                throttle_duration_sec=2.0)
            return

        # Başlangıç konumunu kaydet
        robot_pos = self._get_robot_position()
        if robot_pos is None:
            self.get_logger().warn('Robot konumu alınamadı, bekleniyor...')
            return

        self.home_position = robot_pos
        self.get_logger().info(
            f'🏠 Başlangıç konumu kaydedildi: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})')
        self.get_logger().info('🚀 KEŞFE BAŞLANIYOR!')
        self.state = STATE_EXPLORING

    def _handle_exploring(self):
        """Frontier tespit et, küme seç, hedefe git."""
        # Nav2 hedefi aktifse bekle
        if self.goal_active:
            return

        if self.current_map is None:
            return

        robot_pos = self._get_robot_position()
        if robot_pos is None:
            self.get_logger().warn(
                'Robot konumu alınamadı', throttle_duration_sec=5.0)
            return

        # Frontier tespiti
        frontier_cells = self._detect_frontiers(self.current_map)

        if not frontier_cells:
            self.get_logger().info(
                '✅ Frontier kalmadı — Tüm alan keşfedildi!')
            self.state = STATE_RETURNING_HOME
            self._send_goal_to_home()
            return

        # Kümeleme
        clusters = self._cluster_frontiers(frontier_cells)

        self.get_logger().info(
            f'🔍 Frontier: {len(frontier_cells)} hücre, '
            f'{len(clusters)} küme (min_size={self.min_frontier_size})')

        if not clusters:
            self.get_logger().info(
                '✅ Yeterli boyutta frontier kümesi kalmadı — Keşif tamamlandı!')
            self.state = STATE_RETURNING_HOME
            self._send_goal_to_home()
            return

        # Frontier görselleştirme
        self._publish_frontier_markers(clusters)

        # En iyi kümeyi seç
        target, target_cluster = self._select_best_cluster(
            clusters, robot_pos)

        if target is None:
            self.get_logger().info(
                '✅ Ulaşılabilir frontier kalmadı — Keşif tamamlandı!')
            self.state = STATE_RETURNING_HOME
            self._send_goal_to_home()
            return

        # Seçilen kümeyi görselleştir
        self.current_target_cluster = target_cluster
        self._publish_target_markers(target_cluster)

        # Nav2'ye hedef gönder
        self.current_target = target
        robot_dist = math.sqrt(
            (target[0] - robot_pos[0]) ** 2 +
            (target[1] - robot_pos[1]) ** 2)
        self.get_logger().info(
            f'🎯 Yeni hedef: ({target[0]:.2f}, {target[1]:.2f}) — '
            f'Mesafe: {robot_dist:.2f}m, Küme: {len(target_cluster)} hücre')
        self._send_nav2_goal(target[0], target[1])

    def _handle_returning_home(self):
        """Başlangıç konumuna dönüş kontrolü."""
        if self.goal_active:
            return  # Nav2 hedefe götürüyor, bekle

        # Hedefe ulaşıldı mı kontrol et (goal callback'ten gelecek)
        # Bu noktaya goal başarıyla tamamlandığında veya başarısız olduğunda gelir
        robot_pos = self._get_robot_position()
        if robot_pos is None:
            return

        dist = math.sqrt(
            (robot_pos[0] - self.home_position[0]) ** 2 +
            (robot_pos[1] - self.home_position[1]) ** 2)

        if dist < self.home_tolerance:
            self.get_logger().info('🏠 Başlangıç konumuna ulaşıldı — İniş başlatılıyor')
            self.state = STATE_COMPLETED
            self._send_land_command()
        else:
            self.get_logger().info(
                f'🏠 Eve dönüş tekrar deneniyor (mesafe: {dist:.2f}m)')
            self._send_goal_to_home()

    # ─────────────────────────────────────────────────────────
    # Frontier Tespit Algoritması
    # ─────────────────────────────────────────────────────────

    def _detect_frontiers(self, map_msg):
        """OccupancyGrid üzerinde frontier hücreleri tespit et.

        Frontier = Boş (free=0) hücreye komşu bilinmeyen (unknown=-1) hücre sınırı.
        Engel hücrelerine bitişik frontier'lar güvenlik için filtrelenir.
        """
        width = map_msg.info.width
        height = map_msg.info.height
        data = list(map_msg.data)
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        frontier_cells = []

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x

                # Sadece boş hücreleri kontrol et
                if data[idx] != CELL_FREE:
                    continue

                is_frontier = False
                near_obstacle = False

                for dy, dx in NEIGHBORS_8:
                    ny, nx = y + dy, x + dx
                    nidx = ny * width + nx
                    val = data[nidx]

                    if val == CELL_UNKNOWN:
                        is_frontier = True
                    elif val > 50:  # Engel (yüksek değer)
                        near_obstacle = True

                # Frontier ama engele yakın değil → geçerli frontier
                if is_frontier and not near_obstacle:
                    # Dünya koordinatlarını da sakla
                    wx = origin_x + (x + 0.5) * resolution
                    wy = origin_y + (y + 0.5) * resolution
                    frontier_cells.append((x, y, wx, wy))

        return frontier_cells

    # ─────────────────────────────────────────────────────────
    # Kümeleme Algoritması (BFS Connected Components)
    # ─────────────────────────────────────────────────────────

    def _cluster_frontiers(self, frontier_cells):
        """Frontier hücrelerini BFS ile bağlı bileşenlere ayır.

        Returns:
            List[List[Tuple]]: Her küme, (x, y, wx, wy) tuple'larının listesi.
            min_frontier_size'dan küçük kümeler filtrelenir.
        """
        # Grid koordinatlarından hızlı erişim için set oluştur
        grid_to_cell = {}
        for cell in frontier_cells:
            grid_to_cell[(cell[0], cell[1])] = cell

        visited = set()
        clusters = []

        for cell in frontier_cells:
            grid_key = (cell[0], cell[1])
            if grid_key in visited:
                continue

            # BFS ile bağlı bileşeni bul
            cluster = []
            queue = deque([grid_key])

            while queue:
                current = queue.popleft()
                if current in visited:
                    continue
                visited.add(current)

                if current in grid_to_cell:
                    cluster.append(grid_to_cell[current])

                    # 8-komşuluktaki frontier hücrelerini kuyruğa ekle
                    cx, cy = current
                    for dy, dx in NEIGHBORS_8:
                        neighbor = (cx + dx, cy + dy)
                        if neighbor in grid_to_cell and neighbor not in visited:
                            queue.append(neighbor)

            # Minimum boyut filtresi
            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)

        return clusters

    # ─────────────────────────────────────────────────────────
    # Küme Seçim Stratejisi
    # ─────────────────────────────────────────────────────────

    def _select_best_cluster(self, clusters, robot_pos):
        """Kümeleme tabanlı hedef seçimi.

        Strateji:
        1. Yakın kümeler varsa (< nearby_threshold) → en büyüğünü seç
        2. Yakın küme yoksa → skor = boyut / mesafe ile en iyisini seç
        3. Kara listedeki hedeflere yakın centroid'leri atla
        4. Centroid çok yakınsa (< min_goal_distance) → kümenin en uzak noktasını kullan

        Returns:
            Tuple: ((wx, wy), cluster) veya (None, None)
        """
        candidates = []

        for cluster in clusters:
            # Centroid hesapla
            cx = sum(c[2] for c in cluster) / len(cluster)
            cy = sum(c[3] for c in cluster) / len(cluster)

            # Hedef noktayı belirle — centroid veya en uzak nokta
            goal_x, goal_y = cx, cy
            centroid_dist = math.sqrt(
                (cx - robot_pos[0]) ** 2 + (cy - robot_pos[1]) ** 2)

            # Centroid çok yakınsa, kümenin robota en uzak noktasını hedef al
            # Bu, 180° LiDAR ile oluşan "robotun üzerindeki centroid" sorununu çözer
            if centroid_dist < self.min_goal_distance:
                farthest_dist = 0.0
                for cell in cluster:
                    d = math.sqrt(
                        (cell[2] - robot_pos[0]) ** 2 +
                        (cell[3] - robot_pos[1]) ** 2)
                    if d > farthest_dist:
                        farthest_dist = d
                        goal_x, goal_y = cell[2], cell[3]

                # En uzak nokta da çok yakınsa bu kümeyi atla
                if farthest_dist < self.min_goal_distance:
                    continue

            # Kara listede mi kontrol et
            if self._is_blacklisted(goal_x, goal_y):
                continue

            goal_dist = math.sqrt(
                (goal_x - robot_pos[0]) ** 2 +
                (goal_y - robot_pos[1]) ** 2)
            size = len(cluster)

            candidates.append({
                'goal': (goal_x, goal_y),
                'distance': goal_dist,
                'size': size,
                'cluster': cluster
            })

        if not candidates:
            return None, None

        # Yakın kümeleri filtrele
        nearby = [c for c in candidates if c['distance'] < self.nearby_threshold]

        if nearby:
            # Yakın kümeler arasından en büyüğünü seç (yerel alanı bitir)
            best = max(nearby, key=lambda c: c['size'])
        else:
            # Tüm kümeler uzak — skor = boyut / (mesafe^2)
            best = max(candidates, key=lambda c: c['size'] / (c['distance'] ** 2 + 0.1))

        self.get_logger().info(
            f'  Seçilen küme: hedef=({best["goal"][0]:.2f}, {best["goal"][1]:.2f}), '
            f'mesafe={best["distance"]:.2f}m, boyut={best["size"]}')

        return best['goal'], best['cluster']

    # ─────────────────────────────────────────────────────────
    # Nav2 Entegrasyonu
    # ─────────────────────────────────────────────────────────

    def _send_nav2_goal(self, x, y):
        """Nav2 NavigateToPose action hedefi gönder."""
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server bulunamadı!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.goal_active = True
        send_goal_future = self.nav2_client.send_goal_async(
            goal_msg, feedback_callback=self._nav2_feedback_cb)
        send_goal_future.add_done_callback(self._nav2_goal_response_cb)

    def _nav2_goal_response_cb(self, future):
        """Nav2 hedef kabul/red yanıtı."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('❌ Nav2 hedefi reddetti')
            self.goal_active = False
            if self.current_target:
                self._add_to_blacklist(
                    self.current_target[0], self.current_target[1])
            return

        self.get_logger().info('✅ Nav2 hedefi kabul etti')
        self.goal_handle = goal_handle

        # Sonucu bekle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_cb)

    def _nav2_feedback_cb(self, feedback_msg):
        """Nav2 navigasyon ilerleme geri bildirimi."""
        pass  # İlerleme bilgisi şu an için kullanılmıyor

    def _nav2_result_cb(self, future):
        """Nav2 navigasyon sonucu."""
        result = future.result()
        status = result.status

        self.goal_active = False
        self.goal_handle = None

        # ActionGoalStatus: SUCCEEDED=4, ABORTED=6, CANCELED=5
        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f'✅ Hedefe ulaşıldı: ({self.current_target[0]:.2f}, '
                f'{self.current_target[1]:.2f})')
        elif status == 6:  # ABORTED
            self.get_logger().warn(
                f'⚠️ Hedefe ulaşılamadı (ABORTED): ({self.current_target[0]:.2f}, '
                f'{self.current_target[1]:.2f})')
            if self.current_target:
                self._add_to_blacklist(
                    self.current_target[0], self.current_target[1])
        elif status == 5:  # CANCELED
            self.get_logger().info('🚫 Hedef iptal edildi')
        else:
            self.get_logger().warn(f'⚠️ Beklenmeyen hedef durumu: {status}')
            if self.current_target:
                self._add_to_blacklist(
                    self.current_target[0], self.current_target[1])

        self.current_target = None

    def _send_goal_to_home(self):
        """Başlangıç konumunu Nav2 hedefi olarak gönder."""
        if self.home_position is None:
            self.get_logger().error('Başlangıç konumu kayıtlı değil!')
            return

        self.get_logger().info(
            f'🏠 Eve dönüş başlatıldı: ({self.home_position[0]:.2f}, '
            f'{self.home_position[1]:.2f})')
        self.current_target = self.home_position
        self._send_nav2_goal(self.home_position[0], self.home_position[1])

    # ─────────────────────────────────────────────────────────
    # Kara Liste Yönetimi
    # ─────────────────────────────────────────────────────────

    def _add_to_blacklist(self, x, y):
        """Başarısız hedefi kara listeye ekle."""
        self.blacklisted_goals.append((x, y))
        self.get_logger().info(
            f'🚫 Kara listeye eklendi: ({x:.2f}, {y:.2f}) — '
            f'Toplam: {len(self.blacklisted_goals)}')

    def _is_blacklisted(self, x, y):
        """Verilen koordinat kara listedeki bir hedefe yakın mı?"""
        for bx, by in self.blacklisted_goals:
            dist = math.sqrt((x - bx) ** 2 + (y - by) ** 2)
            if dist < self.blacklist_radius:
                return True
        return False

    # ─────────────────────────────────────────────────────────
    # İniş Komutu
    # ─────────────────────────────────────────────────────────

    def _send_land_command(self):
        """autonomous.py'ye iniş komutu gönder."""
        msg = String()
        msg.data = 'LAND'
        self.command_pub.publish(msg)
        self.get_logger().info('🛬 İNİŞ komutu gönderildi → /explore/command')

    # ─────────────────────────────────────────────────────────
    # Robot Konumu (TF2)
    # ─────────────────────────────────────────────────────────

    def _get_robot_position(self):
        """TF2 ile dronun map frame'deki konumunu al.

        Returns:
            Tuple[float, float] veya None: (x, y) map koordinatları
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return (x, y)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f'TF hatası: {e}')
            return None

    # ─────────────────────────────────────────────────────────
    # RViz Görselleştirme
    # ─────────────────────────────────────────────────────────

    def _publish_frontier_markers(self, clusters):
        """Tüm frontier noktalarını mavi marker olarak yayınla."""
        marker_array = MarkerArray()

        # Önce eski marker'ları temizle
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        marker_array.markers.append(delete_marker)

        marker_id = 0
        for cluster in clusters:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = marker_id
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            # Mavi renk
            marker.color.r = 0.2
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 0.7

            for cell in cluster:
                from geometry_msgs.msg import Point
                p = Point()
                p.x = float(cell[2])
                p.y = float(cell[3])
                p.z = 0.5  # Harita seviyesinin biraz üstünde göster
                marker.points.append(p)

            marker_array.markers.append(marker)
            marker_id += 1

        self.frontiers_pub.publish(marker_array)

    def _publish_target_markers(self, cluster):
        """Seçilen hedef kümeyi yeşil marker olarak yayınla."""
        marker_array = MarkerArray()

        # Önce eski marker'ları temizle
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        marker_array.markers.append(delete_marker)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        # Yeşil renk
        marker.color.r = 0.2
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.9

        for cell in cluster:
            from geometry_msgs.msg import Point
            p = Point()
            p.x = float(cell[2])
            p.y = float(cell[3])
            p.z = 0.5
            marker.points.append(p)

        marker_array.markers.append(marker)

        # Centroid'i büyük bir küre olarak göster
        cx = sum(c[2] for c in cluster) / len(cluster)
        cy = sum(c[3] for c in cluster) / len(cluster)

        centroid_marker = Marker()
        centroid_marker.header.frame_id = 'map'
        centroid_marker.header.stamp = self.get_clock().now().to_msg()
        centroid_marker.ns = 'target_centroid'
        centroid_marker.id = 1
        centroid_marker.type = Marker.SPHERE
        centroid_marker.action = Marker.ADD
        centroid_marker.pose.position.x = float(cx)
        centroid_marker.pose.position.y = float(cy)
        centroid_marker.pose.position.z = 0.8
        centroid_marker.scale.x = 0.3
        centroid_marker.scale.y = 0.3
        centroid_marker.scale.z = 0.3
        # Parlak yeşil
        centroid_marker.color.r = 0.0
        centroid_marker.color.g = 1.0
        centroid_marker.color.b = 0.0
        centroid_marker.color.a = 1.0

        marker_array.markers.append(centroid_marker)

        self.target_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Frontier Explorer durduruluyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
