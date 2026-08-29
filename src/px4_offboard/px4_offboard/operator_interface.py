"""
Operator Interface — Drone Kontrol Web Arayüzü

Localhost üzerinden çalışan bir web arayüzü ile drone'u kontrol etmeye
olanak tanıyan ROS 2 düğümü. Dahili HTTP sunucusu ile REST API sağlar.

Yayınlar (Publishers):
  /operator/command (std_msgs/String) — ARM, LAND, MANUAL, AUTO, EMERGENCY_STOP

Abonelikler (Subscribers):
  /drone/status        (std_msgs/String) — Drone durum bilgisi (JSON)
  /drone/system_status (std_msgs/String) — Sistem metrikleri (JSON)

HTTP API:
  GET  /api/status  — Son drone durum bilgisi
  GET  /api/system  — Son sistem metrikleri
  POST /api/command — Komut gönder {"command": "ARM"}
"""

import os
import json
import threading
import time
from functools import partial
from http.server import HTTPServer, SimpleHTTPRequestHandler

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


# ─────────────────────────────────────────────────────────
# Geçerli operatör komutları
# ─────────────────────────────────────────────────────────
VALID_COMMANDS = {'ARM', 'LAND', 'MANUAL', 'AUTO', 'EMERGENCY_STOP'}


class OperatorHandler(SimpleHTTPRequestHandler):
    """
    HTTP istek işleyici — REST API ve statik dosya sunucusu.

    API Endpoints:
      GET  /api/status  → Drone durum JSON
      GET  /api/system  → Sistem metrikleri JSON
      POST /api/command → Komut gönder
      GET  /*           → Statik dosya (web dizininden)
    """

    def __init__(self, *args, ros_node=None, web_dir=None, **kwargs):
        self.ros_node = ros_node
        self._web_dir = web_dir
        super().__init__(*args, directory=web_dir, **kwargs)

    # ─── Yardımcı Metodlar ───

    def _send_json_response(self, status_code: int, data: dict) -> None:
        """JSON yanıt gönder."""
        response_body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(response_body)))
        self._send_cors_headers()
        self.end_headers()
        self.wfile.write(response_body)

    def _send_cors_headers(self) -> None:
        """CORS başlıklarını ekle (localhost erişimi için)."""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    # ─── HTTP Metodları ───

    def do_OPTIONS(self):
        """CORS preflight isteklerini yanıtla."""
        self.send_response(204)
        self._send_cors_headers()
        self.end_headers()

    def do_GET(self):
        """GET isteklerini işle — API veya statik dosya."""
        if self.path == '/api/status':
            self._handle_get_status()
        elif self.path == '/api/system':
            self._handle_get_system()
        else:
            # Statik dosya sunucusu (SimpleHTTPRequestHandler)
            super().do_GET()

    def do_POST(self):
        """POST isteklerini işle — Komut gönderme."""
        if self.path == '/api/command':
            self._handle_post_command()
        else:
            self._send_json_response(404, {'error': 'Bilinmeyen endpoint'})

    # ─── API İşleyicileri ───

    def _handle_get_status(self) -> None:
        """GET /api/status — Son drone durum bilgisini döndür."""
        if self.ros_node is None:
            self._send_json_response(503, {'error': 'ROS düğümü hazır değil'})
            return

        status_data = self.ros_node.get_drone_status()
        self._send_json_response(200, status_data)

    def _handle_get_system(self) -> None:
        """GET /api/system — Son sistem metriklerini döndür."""
        if self.ros_node is None:
            self._send_json_response(503, {'error': 'ROS düğümü hazır değil'})
            return

        system_data = self.ros_node.get_system_status()
        self._send_json_response(200, system_data)

    def _handle_post_command(self) -> None:
        """POST /api/command — Operatör komutu al ve yayınla."""
        if self.ros_node is None:
            self._send_json_response(503, {'error': 'ROS düğümü hazır değil'})
            return

        try:
            # İstek gövdesini oku
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self._send_json_response(400, {'error': 'Boş istek gövdesi'})
                return

            raw_body = self.rfile.read(content_length)
            body = json.loads(raw_body.decode('utf-8'))
        except (json.JSONDecodeError, ValueError) as e:
            self._send_json_response(400, {
                'error': 'Geçersiz JSON formatı',
                'detail': str(e)
            })
            return

        # Komut doğrulama
        command = body.get('command', '').strip().upper()
        if not command:
            self._send_json_response(400, {'error': 'Komut belirtilmedi'})
            return

        if command not in VALID_COMMANDS:
            self._send_json_response(400, {
                'error': f'Geçersiz komut: {command}',
                'valid_commands': list(VALID_COMMANDS)
            })
            return

        # Komutu ROS topic'e yayınla
        success = self.ros_node.publish_command(command)
        if success:
            self._send_json_response(200, {
                'status': 'ok',
                'command': command,
                'message': f'{command} komutu gönderildi'
            })
        else:
            self._send_json_response(500, {
                'error': 'Komut yayınlanamadı'
            })

    def log_message(self, format, *args):
        """HTTP log mesajlarını ROS logger'a yönlendir (spam'i azalt)."""
        # API çağrılarını logla, statik dosya isteklerini sessizce geç
        msg = format % args
        if '/api/' in msg:
            if self.ros_node is not None:
                self.ros_node.get_logger().debug(f'HTTP: {msg}')


class OperatorInterface(Node):
    """
    Operatör Arayüzü ROS 2 Düğümü.

    Web tabanlı kontrol arayüzü ile drone'u yönetmek için
    HTTP sunucusu ve ROS 2 topic'leri arasında köprü kurar.
    """

    def __init__(self):
        super().__init__('operator_interface')

        # ─── Parametreler ───
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('web_package', 'drone_bringup')

        self.http_port = self.get_parameter('http_port').value
        self.web_package = self.get_parameter('web_package').value

        # ─── Durum Depolama (thread-safe) ───
        self._status_lock = threading.Lock()
        self._drone_status = {}       # Son /drone/status mesajı
        self._system_status = {}      # Son /drone/system_status mesajı
        self._last_status_time = 0.0
        self._last_system_time = 0.0

        # ─── Publisher: Operatör Komutları ───
        self.command_pub = self.create_publisher(
            String, '/operator/command', 10
        )

        # ─── Subscriber: Drone Durumu ───
        self.create_subscription(
            String, '/drone/status',
            self._drone_status_cb, 10
        )

        # ─── Subscriber: Sistem Metrikleri ───
        self.create_subscription(
            String, '/drone/system_status',
            self._system_status_cb, 10
        )

        # ─── Web dizinini bul ───
        self._web_dir = self._find_web_directory()

        # ─── HTTP Sunucusu Başlat ───
        self._http_server = None
        self._http_thread = None
        self._start_http_server()

        self.get_logger().info(
            f'Operator Interface başlatıldı — http://localhost:{self.http_port}'
        )

    # ─────────────────────────────────────────────────────────
    # ROS 2 Callback'ler
    # ─────────────────────────────────────────────────────────

    def _drone_status_cb(self, msg: String) -> None:
        """Drone durum mesajını depola (JSON)."""
        try:
            data = json.loads(msg.data)
            with self._status_lock:
                self._drone_status = data
                self._last_status_time = time.time()
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Geçersiz drone status JSON: {e}')

    def _system_status_cb(self, msg: String) -> None:
        """Sistem metrikleri mesajını depola (JSON)."""
        try:
            data = json.loads(msg.data)
            with self._status_lock:
                self._system_status = data
                self._last_system_time = time.time()
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Geçersiz system status JSON: {e}')

    # ─────────────────────────────────────────────────────────
    # Veri Erişim Metodları (HTTP handler tarafından çağrılır)
    # ─────────────────────────────────────────────────────────

    def get_drone_status(self) -> dict:
        """Son drone durum verisini döndür (thread-safe)."""
        with self._status_lock:
            data = self._drone_status.copy()
            data['_last_update'] = self._last_status_time
            data['_age_seconds'] = round(
                time.time() - self._last_status_time, 1
            ) if self._last_status_time > 0 else -1
            return data

    def get_system_status(self) -> dict:
        """Son sistem metrikleri verisini döndür (thread-safe)."""
        with self._status_lock:
            data = self._system_status.copy()
            data['_last_update'] = self._last_system_time
            data['_age_seconds'] = round(
                time.time() - self._last_system_time, 1
            ) if self._last_system_time > 0 else -1
            return data

    def publish_command(self, command: str) -> bool:
        """Operatör komutunu /operator/command topic'ine yayınla."""
        try:
            msg = String()
            msg.data = command
            self.command_pub.publish(msg)
            self.get_logger().info(f'📡 Operatör komutu gönderildi: {command}')
            return True
        except Exception as e:
            self.get_logger().error(f'Komut yayınlama hatası: {e}')
            return False

    # ─────────────────────────────────────────────────────────
    # HTTP Sunucu Yönetimi
    # ─────────────────────────────────────────────────────────

    def _find_web_directory(self) -> str:
        """Web dosyalarının bulunduğu dizini tespit et."""
        # 1. Önce ament paket dizininden dene
        try:
            pkg_share = get_package_share_directory(self.web_package)
            web_dir = os.path.join(pkg_share, 'web')
            if os.path.isdir(web_dir):
                self.get_logger().info(f'Web dizini (paket): {web_dir}')
                return web_dir
        except Exception:
            self.get_logger().warn(
                f'{self.web_package} paketi bulunamadı, '
                'kaynak dizininden aranıyor...'
            )

        # 2. Kaynak dizininden dene (geliştirme ortamı)
        source_dir = os.path.join(
            os.path.expanduser('~'),
            'drone_project', 'src', 'drone_bringup', 'web'
        )
        if os.path.isdir(source_dir):
            self.get_logger().info(f'Web dizini (kaynak): {source_dir}')
            return source_dir

        # 3. Bulunamazsa mevcut dizini kullan
        self.get_logger().warn(
            'Web dizini bulunamadı! Mevcut dizin kullanılıyor.'
        )
        return os.getcwd()

    def _start_http_server(self) -> None:
        """HTTP sunucusunu daemon thread olarak başlat."""
        try:
            # Handler'ı ROS node ve web dizini ile yapılandır
            handler = partial(
                OperatorHandler,
                ros_node=self,
                web_dir=self._web_dir
            )

            self._http_server = HTTPServer(
                ('0.0.0.0', self.http_port), handler
            )

            # Daemon thread — ana process kapanınca otomatik sonlanır
            self._http_thread = threading.Thread(
                target=self._http_server.serve_forever,
                daemon=True,
                name='operator_http_server'
            )
            self._http_thread.start()

            self.get_logger().info(
                f'HTTP sunucusu başlatıldı — '
                f'Port: {self.http_port}, '
                f'Web dizini: {self._web_dir}'
            )

        except OSError as e:
            self.get_logger().error(
                f'HTTP sunucusu başlatılamadı (port {self.http_port}): {e}'
            )
            self.get_logger().error(
                'Port kullanılıyor olabilir. '
                'Farklı port denemek için: '
                '--ros-args -p http_port:=8081'
            )
            raise

    def _stop_http_server(self) -> None:
        """HTTP sunucusunu durdur."""
        if self._http_server is not None:
            self.get_logger().info('HTTP sunucusu durduruluyor...')
            self._http_server.shutdown()
            self._http_server.server_close()
            self.get_logger().info('HTTP sunucusu durduruldu.')

    def destroy_node(self):
        """Düğüm kapatılırken HTTP sunucusunu da durdur."""
        self._stop_http_server()
        super().destroy_node()


def main(args=None):
    """Operator Interface düğümünü başlat."""
    rclpy.init(args=args)

    node = OperatorInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt — kapatılıyor...')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
