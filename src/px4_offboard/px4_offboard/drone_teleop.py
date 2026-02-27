"""
Drone Teleop — Klavye ile drone kontrolü
─────────────────────────────────────────
Basit bir test aracı. Klavyeden waypoint gönderir.

Kontroller:
  w/s     : İleri / Geri (x ekseni)
  a/d     : Sol / Sağ (y ekseni)
  q/e     : Yukarı / Aşağı (z ekseni)
  t       : Takeoff (arm + offboard + kalkış)
  l       : Land (iniş)
  SPACE   : Yerinde dur (hover)
  Ctrl+C  : Çıkış

Kullanım:
  ros2 run px4_offboard drone_teleop
"""

import sys
import termios
import tty

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


HELP_TEXT = """
╔═══════════════════════════════════════════╗
║         Drone Teleop Kontrolü             ║
╠═══════════════════════════════════════════╣
║  w/s : İleri (+x) / Geri (-x)  [0.5m]   ║
║  a/d : Sol (+y) / Sağ (-y)     [0.5m]   ║
║  q/e : Yukarı / Aşağı          [0.3m]   ║
║  t   : Takeoff                           ║
║  l   : Land                              ║
║  SPC : Hover (yerinde dur)               ║
║  ESC : Çıkış                             ║
╚═══════════════════════════════════════════╝
"""


def get_key():
    """Tek tuş okuma (blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class DroneTeleop(Node):
    def __init__(self):
        super().__init__("drone_teleop")

        self.declare_parameter("step_xy", 0.5)
        self.declare_parameter("step_z", 0.3)
        self.declare_parameter("takeoff_height", 1.5)

        self.step_xy = self.get_parameter("step_xy").value
        self.step_z = self.get_parameter("step_z").value
        self.takeoff_height = self.get_parameter("takeoff_height").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.pos_sub = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._pos_cb, qos)
        self.status_sub = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self._status_cb, qos)

        self.position = [0.0, 0.0, 0.0]
        self.target = [0.0, 0.0, -self.takeoff_height]
        self.armed = False
        self.offboard = False

        # 20 Hz offboard heartbeat
        self.timer = self.create_timer(0.05, self._heartbeat)

        self.get_logger().info("Drone Teleop başlatıldı")
        print(HELP_TEXT)

    def _pos_cb(self, msg):
        self.position = [msg.x, msg.y, msg.z]

    def _status_cb(self, msg):
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _heartbeat(self):
        """Offboard modunu canlı tut."""
        mode = OffboardControlMode()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.position = self.target.copy()
        sp.yaw = float("nan")
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(sp)

    def _send_cmd(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.param1 = p1
        msg.param2 = p2
        msg.command = cmd
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def takeoff(self):
        self.target = [0.0, 0.0, -self.takeoff_height]
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info(f"TAKEOFF → yükseklik {self.takeoff_height}m")

    def land_cmd(self):
        self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("LAND komutu gönderildi")

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                key = get_key()

                if key == "t":
                    self.takeoff()
                elif key == "l":
                    self.land_cmd()
                elif key == "w":
                    self.target[0] += self.step_xy
                    print(f"  → İleri  | Hedef: x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                elif key == "s":
                    self.target[0] -= self.step_xy
                    print(f"  → Geri   | Hedef: x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                elif key == "a":
                    self.target[1] += self.step_xy
                    print(f"  → Sol    | Hedef: x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                elif key == "d":
                    self.target[1] -= self.step_xy
                    print(f"  → Sağ    | Hedef: x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                elif key == "q":
                    self.target[2] -= self.step_z  # NED: z negatif = yukarı
                    print(f"  → Yukarı | Hedef: x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                elif key == "e":
                    self.target[2] += self.step_z
                    print(f"  → Aşağı  | Hedef: x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                elif key == " ":
                    self.target = list(self.position)
                    print(f"  → HOVER  | Pozisyon: x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                elif key == "\x1b" or key == "\x03":  # ESC or Ctrl+C
                    break
        except Exception as e:
            self.get_logger().error(f"Hata: {e}")
        finally:
            self.land_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleop()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
