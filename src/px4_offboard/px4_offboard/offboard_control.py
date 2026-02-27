"""
PX4 Offboard Control Node
──────────────────────────
PX4 ile ROS 2 üzerinden haberleşerek drone'u offboard modda kontrol eder.

Temel görevler:
  1. Offboard modu etkinleştirme
  2. Arm (motorları çalıştırma)
  3. Takeoff (belirli yüksekliğe çıkma)
  4. Waypoint'e gitme (position setpoint)
  5. Land (iniş)

Kullanım:
  ros2 run px4_offboard offboard_control
  ros2 run px4_offboard offboard_control --ros-args -p takeoff_height:=2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleOdometry,
)

import math
from enum import Enum, auto


class DroneState(Enum):
    """Drone durum makinesi."""
    IDLE = auto()
    ARMING = auto()
    TAKEOFF = auto()
    HOVER = auto()
    NAVIGATE = auto()
    LAND = auto()
    LANDED = auto()


class OffboardControl(Node):
    """PX4 Offboard kontrol node'u."""

    def __init__(self):
        super().__init__("offboard_control")

        # ── Parametreler ──
        self.declare_parameter("takeoff_height", 1.5)  # metre (kapalı alan için düşük)
        self.declare_parameter("position_threshold", 0.3)  # metre
        self.declare_parameter("offboard_setpoint_counter", 10)

        self.takeoff_height = self.get_parameter("takeoff_height").value
        self.position_threshold = self.get_parameter("position_threshold").value
        self.setpoint_counter_threshold = self.get_parameter("offboard_setpoint_counter").value

        # ── QoS ──
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ──
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )

        # ── Subscribers ──
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos_profile,
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile,
        )

        # ── State ──
        self.state = DroneState.IDLE
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Hedef pozisyon (NED frame: x=kuzey, y=doğu, z=aşağı)
        self.target_position = [0.0, 0.0, -self.takeoff_height]

        # Waypoint listesi (NED)
        self.waypoints = []
        self.current_waypoint_index = 0

        # ── Timer: 50 Hz kontrol döngüsü ──
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info(
            f"Offboard kontrol başlatıldı | "
            f"Takeoff yüksekliği: {self.takeoff_height}m"
        )

    # ──────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    # ──────────────────────────────────────────────
    # PX4 Komutları
    # ──────────────────────────────────────────────

    def publish_offboard_control_mode(self, position=True, velocity=False, acceleration=False):
        """Offboard kontrol modu yayınla."""
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = acceleration
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x: float, y: float, z: float, yaw: float = float("nan")):
        """Hedef pozisyon yayınla (NED frame)."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        """PX4 araç komutu gönder."""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        """Motorları çalıştır."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("ARM komutu gönderildi")

    def disarm(self):
        """Motorları kapat."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info("DISARM komutu gönderildi")

    def engage_offboard_mode(self):
        """Offboard modu etkinleştir."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("OFFBOARD mod komutu gönderildi")

    def land(self):
        """İniş komutu."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("LAND komutu gönderildi")

    # ──────────────────────────────────────────────
    # Yardımcı Fonksiyonlar
    # ──────────────────────────────────────────────

    def distance_to_target(self) -> float:
        """Mevcut pozisyon ile hedef arasındaki mesafe (3D)."""
        pos = self.vehicle_local_position
        dx = pos.x - self.target_position[0]
        dy = pos.y - self.target_position[1]
        dz = pos.z - self.target_position[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def has_reached_target(self) -> bool:
        """Hedefe ulaşıldı mı?"""
        return self.distance_to_target() < self.position_threshold

    def set_waypoints(self, waypoints: list):
        """
        Waypoint listesi ayarla.
        Her waypoint: [x, y, z] (NED frame, z negatif = yukarı)
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.get_logger().info(f"{len(waypoints)} waypoint yüklendi")

    # ──────────────────────────────────────────────
    # Ana Kontrol Döngüsü
    # ──────────────────────────────────────────────

    def timer_callback(self):
        """50 Hz kontrol döngüsü — durum makinesi."""

        # Her zaman offboard control mode gönder
        self.publish_offboard_control_mode(position=True)

        if self.state == DroneState.IDLE:
            # Offboard modu başlatmak için yeterli setpoint gönder
            self.publish_trajectory_setpoint(0.0, 0.0, -self.takeoff_height)
            self.offboard_setpoint_counter += 1

            if self.offboard_setpoint_counter >= self.setpoint_counter_threshold:
                self.engage_offboard_mode()
                self.arm()
                self.state = DroneState.ARMING
                self.get_logger().info("→ ARMING durumuna geçiliyor")

        elif self.state == DroneState.ARMING:
            self.publish_trajectory_setpoint(0.0, 0.0, -self.takeoff_height)

            # Arming durumunu kontrol et
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.state = DroneState.TAKEOFF
                self.target_position = [0.0, 0.0, -self.takeoff_height]
                self.get_logger().info(
                    f"→ TAKEOFF durumuna geçiliyor (hedef: {self.takeoff_height}m)"
                )

        elif self.state == DroneState.TAKEOFF:
            self.publish_trajectory_setpoint(*self.target_position)

            if self.has_reached_target():
                self.state = DroneState.HOVER
                self.get_logger().info(
                    f"✓ Takeoff tamamlandı | Yükseklik: {-self.vehicle_local_position.z:.2f}m"
                )
                self.get_logger().info("→ HOVER durumuna geçiliyor")

                # Eğer waypoint varsa navigasyona başla
                if self.waypoints:
                    self.state = DroneState.NAVIGATE
                    self.target_position = self.waypoints[0]
                    self.get_logger().info(
                        f"→ NAVIGATE durumuna geçiliyor | "
                        f"Waypoint 1/{len(self.waypoints)}"
                    )

        elif self.state == DroneState.HOVER:
            # Yerinde dur
            self.publish_trajectory_setpoint(*self.target_position)

        elif self.state == DroneState.NAVIGATE:
            self.publish_trajectory_setpoint(*self.target_position)

            if self.has_reached_target():
                self.current_waypoint_index += 1

                if self.current_waypoint_index < len(self.waypoints):
                    self.target_position = self.waypoints[self.current_waypoint_index]
                    self.get_logger().info(
                        f"✓ Waypoint {self.current_waypoint_index}/{len(self.waypoints)} ulaşıldı | "
                        f"Sonraki: {self.target_position}"
                    )
                else:
                    self.get_logger().info("✓ Tüm waypoint'ler tamamlandı → HOVER")
                    self.state = DroneState.HOVER

        elif self.state == DroneState.LAND:
            self.land()
            self.state = DroneState.LANDED

        elif self.state == DroneState.LANDED:
            # İniş tamamlanmasını bekle
            if not self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("✓ İniş tamamlandı, motorlar kapandı")
                self.state = DroneState.IDLE
                self.offboard_setpoint_counter = 0


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C ile durduruluyor...")
        node.land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
