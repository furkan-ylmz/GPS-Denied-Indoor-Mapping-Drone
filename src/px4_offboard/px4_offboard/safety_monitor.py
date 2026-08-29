"""
Güvenlik İzleme Düğümü — Safety Monitor

RPi 5 üzerinde çalışan sensör ve sistem sağlığını izleyen düğüm.
Sensör zaman aşımlarını, CPU/RAM kullanımını ve CPU sıcaklığını takip eder.

Yayınlar:
  /drone/system_status (std_msgs/String, JSON) — 2 Hz
  /drone/warnings (std_msgs/String) — Uyarı durumlarında

İzlenen Sensörler:
  /unilidar/cloud          — Unitree L1 LiDAR sağlığı
  /camera/image_raw        — Pi Camera sağlığı
  /fmu/out/vehicle_status  — PX4 bağlantı durumu
"""

import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus

# PX4 ile uyumlu QoS profili
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Sensör QoS profili (sensör verileri için)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Eşik değerleri
SENSOR_TIMEOUT = 5.0         # Sensör mesaj zaman aşımı (saniye)
CPU_TEMP_WARNING = 80.0      # CPU sıcaklık uyarı eşiği (°C)
CPU_USAGE_WARNING = 90.0     # CPU kullanım uyarı eşiği (%)
RAM_USAGE_WARNING = 85.0     # RAM kullanım uyarı eşiği (%)


class SafetyMonitor(Node):
    """Sensör sağlığı ve sistem kaynaklarını izleyen güvenlik düğümü.

    Standart Python kütüphaneleri ile /proc ve /sys dosya sisteminden
    sistem bilgilerini okur (psutil bağımlılığı yok).
    """

    def __init__(self):
        super().__init__('safety_monitor')

        # ─────────────────────────────────────────────────────────
        # Publisher'lar
        # ─────────────────────────────────────────────────────────

        # Sistem durumu yayını (JSON, 2 Hz)
        self.system_status_pub = self.create_publisher(
            String, '/drone/system_status', 10)

        # Uyarı yayını (sorun olduğunda)
        self.warnings_pub = self.create_publisher(
            String, '/drone/warnings', 10)

        # ─────────────────────────────────────────────────────────
        # Subscriber'lar — Sensör Sağlık İzleme
        # ─────────────────────────────────────────────────────────

        # LiDAR sağlığı
        self.create_subscription(
            PointCloud2, '/unilidar/cloud',
            self._lidar_cb, SENSOR_QOS)

        # Kamera sağlığı
        self.create_subscription(
            Image, '/camera/image_raw',
            self._camera_cb, SENSOR_QOS)

        # PX4 bağlantı durumu
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._px4_cb, PX4_QOS)

        # ─────────────────────────────────────────────────────────
        # Sensör Son Görülme Zamanları
        # ─────────────────────────────────────────────────────────
        self.last_lidar_time = None       # Son LiDAR mesaj zamanı
        self.last_camera_time = None      # Son kamera mesaj zamanı
        self.last_px4_time = None         # Son PX4 mesaj zamanı

        # ─────────────────────────────────────────────────────────
        # CPU İstatistikleri (önceki ölçüm için)
        # ─────────────────────────────────────────────────────────
        self._prev_cpu_times = self._read_cpu_times()

        # ─────────────────────────────────────────────────────────
        # Zamanlayıcılar
        # ─────────────────────────────────────────────────────────

        # Sistem durumu yayını (0.5 Hz — 2 saniye aralıkla)
        self.status_timer = self.create_timer(2.0, self._publish_system_status)

        self.get_logger().info('Güvenlik İzleme düğümü başlatıldı')
        self.get_logger().info(f'  Sensör zaman aşımı: {SENSOR_TIMEOUT}s')
        self.get_logger().info(f'  CPU sıcaklık eşiği: {CPU_TEMP_WARNING}°C')
        self.get_logger().info(f'  CPU kullanım eşiği: {CPU_USAGE_WARNING}%')
        self.get_logger().info(f'  RAM kullanım eşiği: {RAM_USAGE_WARNING}%')

    # ─────────────────────────────────────────────────────────
    # Sensör Callback'leri — Sadece zaman damgası güncelle
    # ─────────────────────────────────────────────────────────

    def _lidar_cb(self, msg):
        """LiDAR mesajı alındı — zamanı güncelle."""
        self.last_lidar_time = time.monotonic()

    def _camera_cb(self, msg):
        """Kamera mesajı alındı — zamanı güncelle."""
        self.last_camera_time = time.monotonic()

    def _px4_cb(self, msg):
        """PX4 durum mesajı alındı — zamanı güncelle."""
        self.last_px4_time = time.monotonic()

    # ─────────────────────────────────────────────────────────
    # Sistem Durum Yayını (her 2 saniye)
    # ─────────────────────────────────────────────────────────

    def _publish_system_status(self):
        """Sistem durumunu JSON formatında yayınla ve uyarıları kontrol et."""
        now = time.monotonic()
        warnings = []

        # --- Sensör Sağlığı Kontrolü ---
        lidar_ok = self._check_sensor(self.last_lidar_time, now, 'LiDAR')
        camera_ok = self._check_sensor(self.last_camera_time, now, 'Kamera')
        px4_ok = self._check_sensor(self.last_px4_time, now, 'PX4')

        if not lidar_ok:
            warnings.append('LiDAR zaman aşımı — mesaj alınmıyor')
        if not camera_ok:
            warnings.append('Kamera zaman aşımı — mesaj alınmıyor')
        if not px4_ok:
            warnings.append('PX4 bağlantısı kesildi — mesaj alınmıyor')

        # --- Sistem Kaynak Kontrolü ---
        cpu_percent = self._get_cpu_percent()
        ram_percent = self._get_ram_percent()
        cpu_temp = self._get_cpu_temp()

        if cpu_temp > CPU_TEMP_WARNING:
            warnings.append(f'CPU sıcaklığı yüksek: {cpu_temp:.1f}°C')
        if cpu_percent > CPU_USAGE_WARNING:
            warnings.append(f'CPU kullanımı yüksek: {cpu_percent:.1f}%')
        if ram_percent > RAM_USAGE_WARNING:
            warnings.append(f'RAM kullanımı yüksek: {ram_percent:.1f}%')

        # --- JSON Durum Yayını ---
        status = {
            'cpu_percent': round(cpu_percent, 1),
            'ram_percent': round(ram_percent, 1),
            'cpu_temp': round(cpu_temp, 1),
            'lidar_ok': lidar_ok,
            'camera_ok': camera_ok,
            'px4_ok': px4_ok,
            'warnings': warnings,
        }

        msg = String()
        msg.data = json.dumps(status)
        self.system_status_pub.publish(msg)

        # --- Uyarı Yayını ---
        if warnings:
            warn_msg = String()
            warn_msg.data = json.dumps(warnings)
            self.warnings_pub.publish(warn_msg)

            for w in warnings:
                self.get_logger().warn(f'⚠️ {w}', throttle_duration_sec=10.0)

    def _check_sensor(self, last_time, now, name):
        """Sensörün zaman aşımına uğrayıp uğramadığını kontrol et.

        Args:
            last_time: Sensörden son mesaj alınma zamanı (monotonic)
            now: Şimdiki zaman (monotonic)
            name: Sensör adı (log için)

        Returns:
            True — sensör sağlıklı, False — zaman aşımı
        """
        if last_time is None:
            # Henüz hiç mesaj alınmadı
            return False
        elapsed = now - last_time
        return elapsed < SENSOR_TIMEOUT

    # ─────────────────────────────────────────────────────────
    # Sistem Kaynak Okuma — /proc ve /sys Dosya Sistemi
    # ─────────────────────────────────────────────────────────

    def _read_cpu_times(self):
        """CPU zamanlarını /proc/stat'tan oku.

        Returns:
            (idle, total) tuple — CPU tick değerleri
        """
        try:
            with open('/proc/stat', 'r') as f:
                # İlk satır: cpu  user nice system idle iowait irq softirq steal guest guest_nice
                line = f.readline()
            parts = line.split()
            # parts[0] = 'cpu', parts[1:] = zaman değerleri
            times = [int(x) for x in parts[1:]]
            idle = times[3]   # idle
            if len(times) > 4:
                idle += times[4]  # iowait
            total = sum(times)
            return (idle, total)
        except (IOError, IndexError, ValueError) as e:
            self.get_logger().warn(f'CPU bilgisi okunamadı: {e}', throttle_duration_sec=30.0)
            return (0, 0)

    def _get_cpu_percent(self):
        """CPU kullanım yüzdesini hesapla (/proc/stat farkı ile).

        Önceki ölçümle aradaki farkı alarak anlık CPU yüzdesi hesaplar.
        """
        current = self._read_cpu_times()
        prev = self._prev_cpu_times
        self._prev_cpu_times = current

        idle_delta = current[0] - prev[0]
        total_delta = current[1] - prev[1]

        if total_delta == 0:
            return 0.0

        cpu_percent = (1.0 - idle_delta / total_delta) * 100.0
        return max(0.0, min(100.0, cpu_percent))

    def _get_ram_percent(self):
        """RAM kullanım yüzdesini /proc/meminfo'dan oku."""
        try:
            meminfo = {}
            with open('/proc/meminfo', 'r') as f:
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        key = parts[0].rstrip(':')
                        value = int(parts[1])  # kB cinsinden
                        meminfo[key] = value

            total = meminfo.get('MemTotal', 0)
            available = meminfo.get('MemAvailable', 0)

            if total == 0:
                return 0.0

            used = total - available
            return (used / total) * 100.0

        except (IOError, ValueError, KeyError) as e:
            self.get_logger().warn(f'RAM bilgisi okunamadı: {e}', throttle_duration_sec=30.0)
            return 0.0

    def _get_cpu_temp(self):
        """CPU sıcaklığını /sys/class/thermal/thermal_zone0/temp'den oku.

        RPi 5'te bu dosya miliderece cinsinden (örn: 58300 = 58.3°C).
        """
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_raw = f.read().strip()
            # miliderece → derece
            return int(temp_raw) / 1000.0
        except (IOError, ValueError) as e:
            self.get_logger().warn(f'CPU sıcaklığı okunamadı: {e}', throttle_duration_sec=30.0)
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Güvenlik İzleme düğümü durduruluyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
