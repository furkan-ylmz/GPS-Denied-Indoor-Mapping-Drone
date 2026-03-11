"""
PX4 Offboard Control Node
──────────────────────────
PX4 ile ROS 2 üzerinden offboard modda drone kontrol eder.

Akış:
  1) 20Hz'de sürekli setpoint gönder (OffboardControlMode + TrajectorySetpoint)
  2) Yeterli setpoint sonrası OFFBOARD mod komutunu gönder
  3) ARM komutunu gönder
  4) Takeoff yüksekliğine çık → HOVER
  5) Waypoint'ler varsa sırayla git
  6) Ctrl+C ile LAND

Kullanım:
  ros2 run px4_offboard offboard_control
  ros2 run px4_offboard offboard_control --ros-args -p takeoff_height:=2.0
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# ── QoS: PX4 uXRCE-DDS best-effort stream ile uyumlu ──
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class State(Enum):
    """Drone durum makinesi."""
    INIT = auto()
    OFFBOARD_REQ = auto()
    ARMING = auto()
    TAKEOFF = auto()
    HOVER = auto()
    NAVIGATE = auto()
    LAND = auto()


class OffboardControl(Node):
    def __init__(self):
        super().__init__("offboard_control")

        # ── Parametreler ──
        self.declare_parameter("takeoff_height", 1.0)
        self.declare_parameter("position_threshold", 0.3)
        self.declare_parameter("setpoint_count_before_offboard", 20)

        self.takeoff_height = self.get_parameter("takeoff_height").value
        self.pos_threshold = self.get_parameter("position_threshold").value
        self.setpoint_threshold = self.get_parameter("setpoint_count_before_offboard").value

        # ── Publishers ──
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)

        # ── Subscribers ──
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
            self._pos_cb, PX4_QOS)
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v2",
            self._status_cb, PX4_QOS)

        # ── Durum ──
        self.state = State.INIT
        self.counter = 0
        self.pos = [0.0, 0.0, 0.0]
        self.armed = False
        self.nav_state = 0
        self.target = [0.0, 0.0, -self.takeoff_height]

        # ── Waypoints ──
        self.waypoints = []
        self.wp_index = 0

        # ── 20 Hz kontrol döngüsü ──
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f"Offboard kontrol baslatildi | takeoff: {self.takeoff_height}m | "
            f"threshold: {self.pos_threshold}m")

    # ────────────── Callbacks ──────────────

    def _pos_cb(self, msg: VehicleLocalPosition):
        self.pos = [msg.x, msg.y, msg.z]

    def _status_cb(self, msg: VehicleStatus):
        old_armed = self.armed
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state
        if self.armed and not old_armed:
            self.get_logger().info("ARM edildi")
        if not self.armed and old_armed:
            self.get_logger().info("DISARM edildi")

    # ────────────── PX4 Komutları ──────────────

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._ts()
        self.offboard_mode_pub.publish(msg)

    def _publish_setpoint(self, x, y, z, yaw=float("nan")):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float("nan"), float("nan"), float("nan")]
        msg.acceleration = [float("nan"), float("nan"), float("nan")]
        msg.yaw = float(yaw)
        msg.timestamp = self._ts()
        self.setpoint_pub.publish(msg)

    def _send_command(self, cmd, p1=0.0, p2=0.0, p7=0.0):
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
        return int(self.get_clock().now().nanoseconds / 1000)

    # ────────────── Yardımcılar ──────────────

    def _dist(self):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(self.pos, self.target)))

    def _at_target(self):
        return self._dist() < self.pos_threshold

    # ────────────── Ana Kontrol Döngüsü ──────────────

    def _control_loop(self):
        # Her döngüde mode + setpoint gönder (offboard kalabilmek için)
        self._publish_offboard_mode()
        self._publish_setpoint(*self.target)

        if self.state == State.INIT:
            self.counter += 1
            if self.counter % 20 == 0:
                self.get_logger().info(
                    f"[INIT] Setpoint gonderiyor ({self.counter}/{self.setpoint_threshold})")
            if self.counter >= self.setpoint_threshold:
                self.get_logger().info("OFFBOARD mod komutu gonderiliyor")
                self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.state = State.OFFBOARD_REQ
                self.counter = 0

        elif self.state == State.OFFBOARD_REQ:
            self.counter += 1
            if self.nav_state == 14:  # NAVIGATION_STATE_OFFBOARD
                self.get_logger().info("OFFBOARD aktif -> ARM gonderiliyor")
                self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.state = State.ARMING
                self.counter = 0
            elif self.counter % 20 == 0:
                self.get_logger().info("[OFFBOARD_REQ] Tekrar gonderiliyor...")
                self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self.counter > 200:
                self.get_logger().error("OFFBOARD moda gecilemedi! PX4 parametrelerini kontrol edin.")
                self.state = State.INIT
                self.counter = 0

        elif self.state == State.ARMING:
            self.counter += 1
            if self.armed:
                self.get_logger().info(f"ARMED -> TAKEOFF ({self.takeoff_height}m)")
                self.state = State.TAKEOFF
                self.counter = 0
            elif self.counter % 20 == 0:
                self.get_logger().info("[ARMING] Tekrar arm gonderiliyor...")
                self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if self.counter > 200:
                self.get_logger().error("ARM basarisiz!")
                self.state = State.INIT
                self.counter = 0

        elif self.state == State.TAKEOFF:
            if self._at_target():
                self.get_logger().info(f"Takeoff tamam | z={-self.pos[2]:.2f}m -> HOVER")
                if self.waypoints:
                    self.target = list(self.waypoints[0])
                    self.wp_index = 0
                    self.state = State.NAVIGATE
                    self.get_logger().info(f"NAVIGATE | WP 1/{len(self.waypoints)}")
                else:
                    self.state = State.HOVER

        elif self.state == State.HOVER:
            pass

        elif self.state == State.NAVIGATE:
            if self._at_target():
                self.wp_index += 1
                if self.wp_index < len(self.waypoints):
                    self.target = list(self.waypoints[self.wp_index])
                    self.get_logger().info(
                        f"WP {self.wp_index}/{len(self.waypoints)} -> {self.target}")
                else:
                    self.get_logger().info("Tum waypoint'ler tamamlandi -> HOVER")
                    self.state = State.HOVER

        elif self.state == State.LAND:
            if not self.armed:
                self.get_logger().info("Inis tamamlandi")
                self.state = State.INIT
                self.counter = 0

    def do_land(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.state = State.LAND
        self.get_logger().info("LAND komutu gonderildi")


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C -> inis yapiliyor...")
        node.do_land()
        import time
        time.sleep(2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
