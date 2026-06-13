"""
Drone Navigator — Nav2 cmd_vel → PX4 TrajectorySetpoint Köprüsü

Nav2'nin ürettiği /cmd_vel (Twist) hız komutlarını alıp PX4'ün
TrajectorySetpoint mesajlarına çeviren ROS 2 düğümü.

Durum Makinesi:
  IDLE → ARMING → TAKING_OFF → NAVIGATING ↔ HOVERING → LANDING

Koordinat Dönüşümü:
  - Nav2 cmd_vel: ENU body frame (base_link) — İleri=x, Sol=y, Yukarı=z
  - PX4 TrajectorySetpoint: NED world frame — Kuzey=x, Doğu=y, Aşağı=z
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# PX4 ile uyumlu QoS profili
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Durum makinesi durumları
STATE_IDLE = 0
STATE_ARMING = 1
STATE_TAKING_OFF = 2
STATE_NAVIGATING = 3
STATE_HOVERING = 4
STATE_LANDING = 5


class DroneNavigator(Node):
    """Nav2 cmd_vel komutlarını PX4 TrajectorySetpoint'e çeviren köprü düğüm."""

    def __init__(self):
        super().__init__('drone_navigator')

        # --- Parametreler ---
        self.target_altitude = -1.5       # NED (negatif = yukarı) → 1.5m yükseklik
        self.takeoff_altitude = -1.5      # Kalkış hedef yüksekliği (NED)
        self.vx_max = 2.0                 # Maks ileri hızı (m/s) — nav2_params.yaml ile senkron
        self.vy_max = 1.5                 # Maks yana hız (m/s) — nav2_params.yaml ile senkron
        self.wz_max = 0.785               # Maks açısal hız (~45°/s) — Odometriyi bozmayan yumuşak dönüş
        self.yaw_deadband = 0.10          # ~6°/s'den küçük açısal hızları yoksay (titreşim filtresi, gerçek dönüşleri geçirir)
        self.cmd_vel_timeout = 0.5        # cmd_vel mesaj zaman aşımı (saniye)
        self.takeoff_threshold = 0.15     # Kalkış tamamlanma eşiği (m)

        # --- Publisher'lar ---
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # --- Subscriber'lar ---
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._pos_cb, PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self._status_cb, PX4_QOS)
        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # Explore modundan iniş komutu
        self.create_subscription(
            String, '/explore/command', self._explore_cmd_cb, 10)

        # Explore modundan aktif hedef bilgisi
        self.create_subscription(
            PoseStamped, '/explore/active_goal', self._active_goal_cb, 10)

        # --- Durum Değişkenleri ---
        self.state = STATE_IDLE
        self.pos = [0.0, 0.0, 0.0]       # Mevcut pozisyon (NED)
        self.current_yaw = 0.0            # Mevcut yaw açısı (NED, rad)
        self.initial_yaw_set = False
        self.armed = False
        self.nav_state = 0
        self.offboard_counter = 0
        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = None
        self.is_yaw_correcting = False
        self.home_position = None         # Başlangıç konumu (NED)
        self.active_goal = None           # [x, y] aktif hedef (map/ENU)

        # --- Zamanlayıcı (20 Hz heartbeat) ---
        self.timer = self.create_timer(0.05, self._heartbeat)

        self.get_logger().info('Drone Navigator başlatıldı — Otonom mod hazır')
        self.get_logger().info(f'  Hedef yükseklik: {abs(self.target_altitude):.1f}m')
        self.get_logger().info(f'  Maks hız: vx={self.vx_max}, vy={self.vy_max} m/s')
        self.get_logger().info(f'  Yaw hızı: {self.wz_max:.3f} rad/s (~{math.degrees(self.wz_max):.0f}°/s), deadband: ~{math.degrees(self.yaw_deadband):.0f}°')

    # ─────────────────────────────────────────────────────────
    # Callback'ler
    # ─────────────────────────────────────────────────────────

    def _pos_cb(self, msg):
        """PX4 yerel pozisyon güncellemesi (NED frame)."""
        self.pos = [msg.x, msg.y, msg.z]
        if not self.initial_yaw_set and not math.isnan(msg.heading):
            self.current_yaw = msg.heading
            self.initial_yaw_set = True
        elif not math.isnan(msg.heading):
            self.current_yaw = msg.heading

    def _status_cb(self, msg):
        """PX4 araç durumu güncellemesi."""
        self.get_logger().info(f'Status received: arming_state={msg.arming_state}')
        old = self.armed
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state
        if self.armed and not old:
            self.get_logger().info('✅ ARM edildi')
        if not self.armed and old:
            self.get_logger().info('🔴 DISARM edildi')

    def _explore_cmd_cb(self, msg):
        """Explore modundan gelen komut (LAND)."""
        if msg.data == 'LAND':
            self.get_logger().info('🛬 İNİŞ komutu alındı — Explore modundan')
            self.state = STATE_LANDING

    def _active_goal_cb(self, msg):
        """Explore modundan gelen aktif hedef koordinatı."""
        if math.isnan(msg.pose.position.x):
            self.active_goal = None
        else:
            self.active_goal = [msg.pose.position.x, msg.pose.position.y]

    def _cmd_vel_cb(self, msg):
        """Nav2'den gelen hız komutu."""
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    # ─────────────────────────────────────────────────────────
    # Ana Döngü (20 Hz)
    # ─────────────────────────────────────────────────────────

    def _heartbeat(self):
        """Durum makinesini çalıştır ve PX4'e setpoint gönder."""
        ts = self._ts()

        if self.state == STATE_IDLE:
            self._handle_idle(ts)
        elif self.state == STATE_ARMING:
            self._handle_arming(ts)
        elif self.state == STATE_TAKING_OFF:
            self._handle_takeoff(ts)
        elif self.state in (STATE_NAVIGATING, STATE_HOVERING):
            self._handle_navigation(ts)
        elif self.state == STATE_LANDING:
            self._handle_landing(ts)

    def _handle_idle(self, ts):
        """IDLE: Setpoint göndermeye başla, yeterli olunca arm et."""
        # Pozisyon modu ile hover setpoint gönder
        self._publish_offboard_mode(ts, position=True, velocity=False)
        self._publish_position_setpoint(ts,
                                        x=self.pos[0], y=self.pos[1],
                                        z=self.takeoff_altitude)

        self.offboard_counter += 1

        if self.offboard_counter >= 20:
            self.get_logger().info('Yeterli setpoint gönderildi, ARMING durumuna geçiliyor...')
            self.state = STATE_ARMING

    def _handle_arming(self, ts):
        """ARMING: Offboard mod ve arm komutlarını gönder."""
        self._publish_offboard_mode(ts, position=True, velocity=False)
        self._publish_position_setpoint(ts,
                                        x=self.pos[0], y=self.pos[1],
                                        z=self.takeoff_altitude)

        if self.offboard_counter % 20 == 0:
            self.get_logger().info('OFFBOARD mod komutu gönderiliyor...')
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            
        self.offboard_counter += 1

        if self.armed:
            self.state = STATE_TAKING_OFF
            self.get_logger().info(f'🚀 KALKIŞ başladı → Hedef: {abs(self.takeoff_altitude):.1f}m')

    def _handle_takeoff(self, ts):
        """TAKING_OFF: Hedef yüksekliğe çıkana kadar pozisyon kontrolü."""
        self._publish_offboard_mode(ts, position=True, velocity=False)
        self._publish_position_setpoint(ts,
                                        x=self.pos[0], y=self.pos[1],
                                        z=self.takeoff_altitude)

        # Hedef yüksekliğe ulaşıldı mı kontrol et
        alt_error = abs(self.pos[2] - self.takeoff_altitude)
        if alt_error < self.takeoff_threshold and self.armed:
            # Başlangıç konumunu kaydet (eve dönüş için)
            self.home_position = [self.pos[0], self.pos[1], self.pos[2]]
            self.get_logger().info('✅ Kalkış tamamlandı — HOVERING durumuna geçiliyor')
            self.get_logger().info(
                f'🏠 Başlangıç konumu kaydedildi: '
                f'({self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f})')
            self.state = STATE_HOVERING

    def _handle_navigation(self, ts):
        """NAVIGATING/HOVERING: cmd_vel komutlarına göre uç veya hover et."""
        now = self.get_clock().now()
        has_cmd_vel = (
            self.last_cmd_vel_time is not None and
            (now - self.last_cmd_vel_time).nanoseconds / 1e9 < self.cmd_vel_timeout
        )

        if has_cmd_vel:
            # Nav2'den aktif hız komutu var — NAVİGASYON modu
            if self.state != STATE_NAVIGATING:
                self.get_logger().info('🧭 NAVİGASYON başladı — cmd_vel takip ediliyor')
                self.state = STATE_NAVIGATING

            self._publish_offboard_mode(ts, position=True, velocity=True)

            cmd = self.last_cmd_vel

            # --- Yöntem 2: Hız Vektörüne Göre Histerezisli Yaw Kontrolü ---
            raw_vx = cmd.linear.x
            raw_vy = cmd.linear.y
            speed = math.sqrt(raw_vx**2 + raw_vy**2)

            vx_body = raw_vx
            vy_body = raw_vy
            wz = 0.0
            yaw_err = 0.0

            # Aktif hedefe yakınlık kontrolü (Süpürme fazı çakışmasını önlemek için)
            is_near_goal = False
            if self.active_goal is not None:
                rx_enu = self.pos[1]  # East
                ry_enu = self.pos[0]  # North
                gx, gy = self.active_goal
                dist = math.sqrt((rx_enu - gx)**2 + (ry_enu - gy)**2)
                if dist < 0.45:  # Hedefe 45 cm kala süpürme/hizalanma aşaması başlar
                    is_near_goal = True

            if speed > 0.02 and not is_near_goal:
                # Hareket halindeyken hız vektörü açısını hesapla (body frame'de yaw hatası)
                yaw_err = math.atan2(raw_vy, raw_vx)
                
                # Histerezis durum geçişleri (45° üstünde düzeltmeye başla, 3° altında durdur)
                if not self.is_yaw_correcting:
                    if abs(yaw_err) > math.radians(45):
                        self.is_yaw_correcting = True
                else:
                    if abs(yaw_err) < math.radians(3):
                        self.is_yaw_correcting = False

                if self.is_yaw_correcting:
                    # P kontrolör ile dönüş hızı üret (Dur ve yerinde dön)
                    wz = 1.0 * yaw_err
                    wz = max(-self.wz_max, min(self.wz_max, wz))
                    
                    # Sapma 45 dereceden büyükse ilerlemeyi durdur (hover & turn)
                    if abs(yaw_err) > math.radians(45):
                        vx_body = 0.0
                        vy_body = 0.0
            else:
                # Dururken, hedefe yaklaşırken veya süpürme yaparken: Nav2'nin kendi yaw komutunu kullan
                if abs(cmd.angular.z) > 0.02:
                    wz = max(-self.wz_max, min(self.wz_max, cmd.angular.z))
                else:
                    wz = 0.0

            # Hız limitleme (Maksimum hızları aşma)
            vx_body = max(-self.vx_max, min(self.vx_max, vx_body))
            vy_body = max(-self.vy_max, min(self.vy_max, vy_body))

            # Body FLU → NED world frame dönüşümü (doğrudan)
            #
            # Nav2 cmd_vel: FLU body frame (Forward=x, Left=y)
            # PX4 setpoint: NED world frame (North=x, East=y, Down=z)
            #
            # FLU→FRD: vx_frd = vx_flu, vy_frd = -vy_flu
            # FRD→NED (heading h, CW from North):
            #   vx_ned = cos(h)*vx_frd - sin(h)*vy_frd
            #   vy_ned = sin(h)*vx_frd + cos(h)*vy_frd
            # Combined (substituting vy_frd = -vy_flu):
            #   vx_ned = cos(h)*vx + sin(h)*vy
            #   vy_ned = sin(h)*vx - cos(h)*vy
            h = self.current_yaw  # NED heading from PX4
            cos_h = math.cos(h)
            sin_h = math.sin(h)
            vx_ned = cos_h * vx_body + sin_h * vy_body
            vy_ned = sin_h * vx_body - cos_h * vy_body

            sp = TrajectorySetpoint()
            sp.velocity = [float(vx_ned), float(vy_ned), float('nan')]
            sp.position = [float('nan'), float('nan'), float(self.target_altitude)]
            sp.yaw = float('nan')
            sp.yawspeed = float(-wz)  # ROS CCW → NED CW: yön ters
            sp.timestamp = ts
            self.setpoint_pub.publish(sp)

        else:
            # cmd_vel timeout — HOVER modu
            if self.state != STATE_HOVERING:
                self.get_logger().info('⏸️  HOVER — cmd_vel zaman aşımı, yerinde duruyorum')
                self.state = STATE_HOVERING

            self._publish_offboard_mode(ts, position=True, velocity=True)

            sp = TrajectorySetpoint()
            sp.velocity = [0.0, 0.0, float('nan')]
            sp.position = [float('nan'), float('nan'), float(self.target_altitude)]
            sp.yaw = float(self.current_yaw)
            sp.yawspeed = float('nan')
            sp.timestamp = ts
            self.setpoint_pub.publish(sp)

    # ─────────────────────────────────────────────────────────
    # İniş Kontrolü
    # ─────────────────────────────────────────────────────────

    def _handle_landing(self, ts):
        """LANDING: PX4 AUTO.LAND moduna geçir."""
        # PX4'e AUTO.LAND modu komutu gönder
        # param1=1.0 (base mode flag), param2=4.0 (PX4_CUSTOM_MAIN_MODE_AUTO),
        # param7 yerine param2 ve p2 kullanarak: mode AUTO, sub-mode LAND
        self._send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            p1=1.0, p2=4.0, p7=6.0)

        self.get_logger().info(
            '🛬 İNİŞ yapılıyor — PX4 AUTO.LAND modu aktif',
            throttle_duration_sec=3.0)

        # İniş tamamlandığında (disarm olduğunda) bildir
        if not self.armed and self.home_position is not None:
            self.get_logger().info('✅ İNİŞ TAMAMLANDI — Dron güvenle indi')
            # Durumu sabit tut, artık heartbeat sadece log basar
            self.state = STATE_LANDING  # Aynı durumda kal

    # ─────────────────────────────────────────────────────────
    # Yardımcı Metodlar
    # ─────────────────────────────────────────────────────────

    def _publish_offboard_mode(self, ts, position=True, velocity=False):
        """OffboardControlMode mesajı yayınla."""
        mode = OffboardControlMode()
        mode.position = position
        mode.velocity = velocity
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = ts
        self.offboard_mode_pub.publish(mode)

    def _publish_position_setpoint(self, ts, x, y, z):
        """Pozisyon setpoint mesajı yayınla."""
        sp = TrajectorySetpoint()
        sp.position = [float(x), float(y), float(z)]
        sp.velocity = [float('nan'), float('nan'), float('nan')]
        sp.acceleration = [float('nan'), float('nan'), float('nan')]
        sp.yaw = float(self.current_yaw)
        sp.timestamp = ts
        self.setpoint_pub.publish(sp)

    def _send_command(self, cmd, p1=0.0, p2=0.0, p7=0.0):
        """PX4 araç komutu gönder."""
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
        """Mikrosaniye cinsinden zaman damgası."""
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = DroneNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Drone Navigator durduruluyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
