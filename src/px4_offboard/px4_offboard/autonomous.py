"""
Drone Navigator — Nav2 cmd_vel → PX4 TrajectorySetpoint Köprüsü

Nav2'nin ürettiği /cmd_vel (Twist) hız komutlarını alıp PX4'ün
TrajectorySetpoint mesajlarına çeviren ROS 2 düğümü.

Durum Makinesi (Gerçek Donanım):
  WAITING_FOR_APPROVAL → IDLE → ARMING → TAKING_OFF → NAVIGATING ↔ HOVERING
    → WAITING_FOR_LAND_APPROVAL → LANDING

Operatör Komutları (/operator/command):
  ARM              — Operatör onayı ile arm sürecini başlat
  LAND             — İniş komutunu onayla
  MANUAL           — Manuel kontrol moduna geç (teleop)
  AUTO             — Otonom navigasyona dön
  EMERGENCY_STOP   — Acil durum, anında disarm

Koordinat Dönüşümü:
  - Nav2 cmd_vel: ENU body frame (base_link) — İleri=x, Sol=y, Yukarı=z
  - PX4 TrajectorySetpoint: NED world frame — Kuzey=x, Doğu=y, Aşağı=z
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from px4_msgs.msg import (
    BatteryStatus,
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
STATE_WAITING_FOR_APPROVAL = 6
STATE_WAITING_FOR_LAND_APPROVAL = 7

# Durum isimleri (JSON status ve log mesajları için)
STATE_NAMES = {
    STATE_IDLE: 'IDLE',
    STATE_ARMING: 'ARMING',
    STATE_TAKING_OFF: 'TAKING_OFF',
    STATE_NAVIGATING: 'NAVIGATING',
    STATE_HOVERING: 'HOVERING',
    STATE_LANDING: 'LANDING',
    STATE_WAITING_FOR_APPROVAL: 'WAITING_FOR_APPROVAL',
    STATE_WAITING_FOR_LAND_APPROVAL: 'WAITING_FOR_LAND_APPROVAL',
}

# Batarya eşikleri
BATTERY_WARNING_PERCENT = 20.0   # Bu seviyenin altında uyarı ve eve dönüş
BATTERY_CRITICAL_PERCENT = 10.0  # Bu seviyenin altında acil iniş


class DroneNavigator(Node):
    """Nav2 cmd_vel komutlarını PX4 TrajectorySetpoint'e çeviren köprü düğüm.

    Gerçek donanım için operatör onay sistemi, manuel mod desteği,
    batarya izleme ve durum yayını eklenmiştir.
    """

    def __init__(self):
        super().__init__('drone_navigator')

        # ─────────────────────────────────────────────────────────
        # ROS Parametreleri (launch dosyasından veya komut satırından ayarlanabilir)
        # ─────────────────────────────────────────────────────────
        self.declare_parameter('target_altitude', -1.5)       # NED hedef yükseklik (negatif = yukarı)
        self.declare_parameter('takeoff_altitude', -1.5)      # Kalkış hedef yüksekliği (NED)
        self.declare_parameter('vx_max', 1.0)                 # Maks ileri hız (m/s) — gerçek donanım için düşürüldü
        self.declare_parameter('vy_max', 0.8)                 # Maks yana hız (m/s) — gerçek donanım için düşürüldü
        self.declare_parameter('wz_max', 0.785)               # Maks açısal hız (~45°/s)
        self.declare_parameter('yaw_deadband', 0.10)          # ~6°/s altı açısal hız filtresi
        self.declare_parameter('cmd_vel_timeout', 0.5)        # cmd_vel mesaj zaman aşımı (saniye)
        self.declare_parameter('takeoff_threshold', 0.15)     # Kalkış tamamlanma eşiği (m)

        # Parametre değerlerini oku
        self.target_altitude = self.get_parameter('target_altitude').value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.vx_max = self.get_parameter('vx_max').value
        self.vy_max = self.get_parameter('vy_max').value
        self.wz_max = self.get_parameter('wz_max').value
        self.yaw_deadband = self.get_parameter('yaw_deadband').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.takeoff_threshold = self.get_parameter('takeoff_threshold').value

        # ─────────────────────────────────────────────────────────
        # Publisher'lar
        # ─────────────────────────────────────────────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # Durum yayını — operatör arayüzü ve diğer düğümler için
        self.status_pub = self.create_publisher(
            String, '/drone/status', 10)

        # ─────────────────────────────────────────────────────────
        # Subscriber'lar
        # ─────────────────────────────────────────────────────────

        # PX4 sensör verileri
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._pos_cb, PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._status_cb, PX4_QOS)
        self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status',
            self._battery_cb, PX4_QOS)

        # Nav2 hız komutları
        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # Teleop hız komutları (manuel mod için)
        self.create_subscription(
            Twist, '/teleop/cmd_vel', self._teleop_cmd_vel_cb, 10)

        # Operatör komutları (ARM, LAND, MANUAL, AUTO, EMERGENCY_STOP)
        self.create_subscription(
            String, '/operator/command', self._operator_cmd_cb, 10)

        # Explore modundan iniş komutu
        self.create_subscription(
            String, '/explore/command', self._explore_cmd_cb, 10)

        # Explore modundan aktif hedef bilgisi
        self.create_subscription(
            PoseStamped, '/explore/active_goal', self._active_goal_cb, 10)

        # ─────────────────────────────────────────────────────────
        # Durum Değişkenleri
        # ─────────────────────────────────────────────────────────
        self.state = STATE_WAITING_FOR_APPROVAL   # Operatör onayı bekle
        self.pos = [0.0, 0.0, 0.0]               # Mevcut pozisyon (NED)
        self.current_yaw = 0.0                    # Mevcut yaw açısı (NED, rad)
        self.initial_yaw_set = False
        self.armed = False
        self.nav_state = 0
        self.offboard_counter = 0
        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = None
        self.is_yaw_correcting = False
        self.home_position = None                 # Başlangıç konumu (NED)
        self.active_goal = None                   # [x, y] aktif hedef (map/ENU)

        # Manuel mod değişkenleri
        self.manual_mode = False                  # Manuel mod bayrağı
        self.last_teleop_cmd = Twist()            # Son teleop hız komutu
        self.last_teleop_time = None              # Son teleop mesaj zamanı

        # Batarya izleme değişkenleri
        self.battery_percent = float('nan')       # Batarya yüzdesi (0-100)
        self.battery_warning_sent = False         # Uyarı gönderildi mi?
        self.battery_critical_sent = False        # Kritik uyarı gönderildi mi?
        self.returning_home = False               # Eve dönüş aktif mi?

        # ─────────────────────────────────────────────────────────
        # Zamanlayıcılar
        # ─────────────────────────────────────────────────────────

        # Ana döngü (20 Hz heartbeat — PX4 setpoint gönderimi)
        self.timer = self.create_timer(0.05, self._heartbeat)

        # Durum yayını zamanlayıcısı (1 Hz — JSON status)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        # ─────────────────────────────────────────────────────────
        # Başlangıç log mesajları
        # ─────────────────────────────────────────────────────────
        self.get_logger().info('Drone Navigator başlatıldı — Operatör onayı bekleniyor')
        self.get_logger().info(f'  Hedef yükseklik: {abs(self.target_altitude):.1f}m')
        self.get_logger().info(f'  Maks hız: vx={self.vx_max}, vy={self.vy_max} m/s')
        self.get_logger().info(
            f'  Yaw hızı: {self.wz_max:.3f} rad/s (~{math.degrees(self.wz_max):.0f}°/s), '
            f'deadband: ~{math.degrees(self.yaw_deadband):.0f}°')
        self.get_logger().info('  Operatör komutu bekleniyor: /operator/command → "ARM"')

    # ─────────────────────────────────────────────────────────
    # Callback'ler — PX4 Sensör Verileri
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
        old = self.armed
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state
        if self.armed and not old:
            self.get_logger().info('✅ ARM edildi')
        if not self.armed and old:
            self.get_logger().info('🔴 DISARM edildi')

    def _battery_cb(self, msg):
        """PX4 batarya durumu güncellemesi.

        BatteryStatus.remaining alanı 0.0–1.0 aralığındadır (1.0 = %100).
        """
        if not math.isnan(msg.remaining):
            self.battery_percent = msg.remaining * 100.0
        else:
            return

        # Kritik batarya seviyesi kontrolü — acil iniş
        if self.battery_percent < BATTERY_CRITICAL_PERCENT:
            if not self.battery_critical_sent:
                self.battery_critical_sent = True
                self.get_logger().error(
                    f'🔴 KRİTİK BATARYA: %{self.battery_percent:.0f} — ACİL İNİŞ BAŞLATILIYOR!')
            # Uçuş halindeyse hemen iniş yap
            if self.state in (STATE_NAVIGATING, STATE_HOVERING,
                              STATE_WAITING_FOR_LAND_APPROVAL):
                self.state = STATE_LANDING
                self.get_logger().error('🔴 Kritik batarya → İNİŞ durumuna geçildi')

        # Düşük batarya seviyesi kontrolü — uyarı ve eve dönüş
        elif self.battery_percent < BATTERY_WARNING_PERCENT:
            if not self.battery_warning_sent:
                self.battery_warning_sent = True
                self.get_logger().warn(
                    f'⚠️ DÜŞÜK BATARYA: %{self.battery_percent:.0f} — Eve dönüş başlatılıyor')
            # Eve dönüş işlemi (home_position kayıtlıysa)
            if not self.returning_home and self.home_position is not None:
                self.returning_home = True
                self.get_logger().warn('⚠️ Eve dönüş rotası aktif')

    # ─────────────────────────────────────────────────────────
    # Callback'ler — Operatör ve Komut Girişleri
    # ─────────────────────────────────────────────────────────

    def _operator_cmd_cb(self, msg):
        """Operatör arayüzünden gelen komutları işle.

        Desteklenen komutlar:
          ARM              — Arm sürecini başlat (sadece WAITING_FOR_APPROVAL durumunda)
          LAND             — İniş komutunu onayla (uçuş durumlarından)
          MANUAL           — Manuel kontrol moduna geç
          AUTO             — Otonom navigasyona dön
          EMERGENCY_STOP   — Acil durum, anında disarm
        """
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'📡 Operatör komutu alındı: {cmd}')

        if cmd == 'ARM':
            if self.state == STATE_WAITING_FOR_APPROVAL:
                self.get_logger().info('✅ Operatör onayı alındı — ARM süreci başlatılıyor')
                self.state = STATE_IDLE
                self.offboard_counter = 0
            else:
                self.get_logger().warn(
                    f'⚠️ ARM komutu reddedildi — Mevcut durum: {STATE_NAMES.get(self.state)}')

        elif cmd == 'LAND':
            if self.state == STATE_WAITING_FOR_LAND_APPROVAL:
                # İniş onayı verildi
                self.get_logger().info('✅ İniş onayı alındı — İNİŞ başlatılıyor')
                self.state = STATE_LANDING
            elif self.state in (STATE_NAVIGATING, STATE_HOVERING):
                # Doğrudan iniş komutu (uçuş sırasında)
                self.get_logger().info('🛬 Operatör iniş komutu — İNİŞ başlatılıyor')
                self.state = STATE_LANDING
            else:
                self.get_logger().warn(
                    f'⚠️ LAND komutu reddedildi — Mevcut durum: {STATE_NAMES.get(self.state)}')

        elif cmd == 'MANUAL':
            if self.state in (STATE_NAVIGATING, STATE_HOVERING,
                              STATE_WAITING_FOR_LAND_APPROVAL):
                self.manual_mode = True
                self.get_logger().info('🎮 MANUEL MOD aktif — Teleop kontrol bekleniyor')
                # Manuel modda hovering durumuna geç
                if self.state != STATE_HOVERING:
                    self.state = STATE_HOVERING
            else:
                self.get_logger().warn(
                    f'⚠️ MANUAL komutu reddedildi — Mevcut durum: {STATE_NAMES.get(self.state)}')

        elif cmd == 'AUTO':
            if self.manual_mode:
                self.manual_mode = False
                self.get_logger().info('🤖 OTONOM MOD aktif — Nav2 navigasyona dönüldü')
            else:
                self.get_logger().info('🤖 Zaten otonom modda')

        elif cmd == 'EMERGENCY_STOP':
            self.get_logger().error('🚨 ACİL DURDURMA — Anında DISARM komutu gönderiliyor!')
            self._send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                p1=0.0,   # 0 = disarm
                p2=21196.0  # force disarm magic number
            )
            self.state = STATE_LANDING
            self.manual_mode = False

        else:
            self.get_logger().warn(f'⚠️ Bilinmeyen operatör komutu: {cmd}')

    def _explore_cmd_cb(self, msg):
        """Explore modundan gelen komut (LAND).

        Gerçek donanımda: doğrudan iniş yerine operatör onayı beklenir.
        """
        if msg.data == 'LAND':
            self.get_logger().info(
                '🛬 İNİŞ talebi alındı — Explore modundan → Operatör onayı bekleniyor')
            # Doğrudan iniş yerine onay bekle
            if self.state in (STATE_NAVIGATING, STATE_HOVERING):
                self.state = STATE_WAITING_FOR_LAND_APPROVAL

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

    def _teleop_cmd_vel_cb(self, msg):
        """Teleop düğümünden gelen hız komutu (manuel mod için)."""
        self.last_teleop_cmd = msg
        self.last_teleop_time = self.get_clock().now()

    # ─────────────────────────────────────────────────────────
    # Durum Yayını (1 Hz)
    # ─────────────────────────────────────────────────────────

    def _publish_status(self):
        """JSON formatında drone durumunu yayınla.

        /drone/status topic'ine yayınlanan bilgiler:
          - state: Durum makinesi durumu
          - armed: Arm durumu
          - altitude: Mevcut yükseklik (m, pozitif = yukarı)
          - position: [x, y, z] NED pozisyon
          - battery_percent: Batarya yüzdesi
          - manual_mode: Manuel mod durumu
        """
        # Yükseklik: NED z negatif = yukarı, ABS ile pozitif göster
        altitude = abs(self.pos[2]) if not math.isnan(self.pos[2]) else 0.0
        battery = round(self.battery_percent, 1) if not math.isnan(self.battery_percent) else -1.0

        status = {
            'state': STATE_NAMES.get(self.state, 'UNKNOWN'),
            'armed': self.armed,
            'altitude': round(altitude, 2),
            'position': [round(self.pos[0], 2), round(self.pos[1], 2), round(self.pos[2], 2)],
            'battery_percent': battery,
            'manual_mode': self.manual_mode,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    # ─────────────────────────────────────────────────────────
    # Ana Döngü (20 Hz)
    # ─────────────────────────────────────────────────────────

    def _heartbeat(self):
        """Durum makinesini çalıştır ve PX4'e setpoint gönder."""
        ts = self._ts()

        if self.state == STATE_WAITING_FOR_APPROVAL:
            self._handle_waiting_for_approval(ts)
        elif self.state == STATE_IDLE:
            self._handle_idle(ts)
        elif self.state == STATE_ARMING:
            self._handle_arming(ts)
        elif self.state == STATE_TAKING_OFF:
            self._handle_takeoff(ts)
        elif self.state in (STATE_NAVIGATING, STATE_HOVERING):
            self._handle_navigation(ts)
        elif self.state == STATE_WAITING_FOR_LAND_APPROVAL:
            self._handle_waiting_for_land_approval(ts)
        elif self.state == STATE_LANDING:
            self._handle_landing(ts)

    # ─────────────────────────────────────────────────────────
    # Durum İşleyicileri
    # ─────────────────────────────────────────────────────────

    def _handle_waiting_for_approval(self, ts):
        """WAITING_FOR_APPROVAL: Operatör ARM onayı bekleniyor.

        Bu durumda PX4'e herhangi bir komut gönderilmez.
        Operatör /operator/command → 'ARM' gönderdiğinde IDLE'a geçilir.
        """
        # Sadece bekleme — operatör onayı _operator_cmd_cb'de işleniyor
        self.get_logger().info(
            '⏳ Operatör onayı bekleniyor — "ARM" komutu gönderin',
            throttle_duration_sec=5.0)

    def _handle_idle(self, ts):
        """IDLE: Setpoint göndermeye başla, yeterli olunca arm et.

        PX4 offboard moda geçmeden önce en az 20 setpoint göndermek gerekir.
        """
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
        """NAVIGATING/HOVERING: cmd_vel komutlarına göre uç veya hover et.

        Manuel modda Nav2 cmd_vel yerine teleop cmd_vel kullanılır.
        """
        now = self.get_clock().now()

        # Manuel modda teleop komutlarını kullan
        if self.manual_mode:
            self._handle_manual_mode(ts, now)
            return

        # Otonom modda Nav2 cmd_vel komutlarını işle
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

            # --- Hız Vektörüne Göre Histerezisli Yaw Kontrolü ---
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

            self._publish_hover_setpoint(ts)

    def _handle_manual_mode(self, ts, now):
        """Manuel modda teleop hız komutlarını PX4'e gönder.

        Teleop komutu zaman aşımına uğrarsa hover modunda kal.
        """
        has_teleop = (
            self.last_teleop_time is not None and
            (now - self.last_teleop_time).nanoseconds / 1e9 < self.cmd_vel_timeout
        )

        if has_teleop:
            # Teleop komutu aktif — hızları dönüştür ve gönder
            self._publish_offboard_mode(ts, position=True, velocity=True)

            cmd = self.last_teleop_cmd
            vx_body = max(-self.vx_max, min(self.vx_max, cmd.linear.x))
            vy_body = max(-self.vy_max, min(self.vy_max, cmd.linear.y))
            wz = max(-self.wz_max, min(self.wz_max, cmd.angular.z))

            # Body FLU → NED world frame dönüşümü (otonom navigasyonla aynı)
            h = self.current_yaw
            cos_h = math.cos(h)
            sin_h = math.sin(h)
            vx_ned = cos_h * vx_body + sin_h * vy_body
            vy_ned = sin_h * vx_body - cos_h * vy_body

            sp = TrajectorySetpoint()
            sp.velocity = [float(vx_ned), float(vy_ned), float('nan')]
            sp.position = [float('nan'), float('nan'), float(self.target_altitude)]
            sp.yaw = float('nan')
            sp.yawspeed = float(-wz)  # ROS CCW → NED CW
            sp.timestamp = ts
            self.setpoint_pub.publish(sp)
        else:
            # Teleop timeout — yerinde hover
            self._publish_hover_setpoint(ts)

    def _handle_waiting_for_land_approval(self, ts):
        """WAITING_FOR_LAND_APPROVAL: Yerinde hover ederek operatör onayı bekle.

        Explore modu veya görev tamamlandığında bu duruma geçilir.
        Operatör 'LAND' komutu gönderdiğinde iniş başlar.
        """
        self.get_logger().info(
            '⏳ İniş onayı bekleniyor — "LAND" komutu gönderin',
            throttle_duration_sec=5.0)

        # Yerinde hover et
        self._publish_hover_setpoint(ts)

    def _publish_hover_setpoint(self, ts):
        """Yerinde hover setpoint'i gönder (hız=0, yükseklik koru, yaw koru)."""
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
