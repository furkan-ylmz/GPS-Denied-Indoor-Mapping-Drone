"""
Drone Teleop — Klavye ile PX4 offboard drone kontrolü.
ROS spin ayrı thread'de çalışır, 20Hz heartbeat hiç kesilmez.
QoS: PX4 uXRCE-DDS best-effort stream uyumlu.
"""

import sys
import termios
import tty
import threading
import select

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


HELP = """
╔═══════════════════════════════════════╗
║       Drone Teleop Kontrolu           ║
╠═══════════════════════════════════════╣
║  t   : Arm + Offboard + Takeoff      ║
║  w/s : Ileri / Geri (+x/-x)  [0.5m]  ║
║  a/d : Sol / Sag   (+y/-y)   [0.5m]  ║
║  q/e : Yukari / Asagi        [0.3m]  ║
║  l   : Land (inis)                    ║
║  SPC : Hover (yerinde dur)            ║
║  ESC : Cikis                          ║
╚═══════════════════════════════════════╝
  Once 't' ile arm+takeoff yapin!
"""


class DroneTeleop(Node):
    def __init__(self):
        super().__init__("drone_teleop")

        self.step_xy = 0.5
        self.step_z = 0.15
        self.takeoff_alt = 1.0

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
        self.pos = [0.0, 0.0, 0.0]
        self.target = [0.0, 0.0, -self.takeoff_alt]
        self.armed = False
        self.nav_state = 0
        self.offboard_counter = 0

        # ── 20 Hz heartbeat ──
        self.timer = self.create_timer(0.05, self._heartbeat)

        self.get_logger().info("Drone Teleop baslatildi")
        print(HELP)

    # ────────────── Callbacks ──────────────

    def _pos_cb(self, msg):
        self.pos = [msg.x, msg.y, msg.z]

    def _status_cb(self, msg):
        old = self.armed
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state
        if self.armed and not old:
            self.get_logger().info("ARM edildi")
        if not self.armed and old:
            self.get_logger().info("DISARM edildi")

    # ────────────── Heartbeat (20Hz) ──────────────

    def _heartbeat(self):
        ts = self._ts()

        mode = OffboardControlMode()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = ts
        self.offboard_mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.position = [float(self.target[0]), float(self.target[1]), float(self.target[2])]
        sp.velocity = [float("nan"), float("nan"), float("nan")]
        sp.acceleration = [float("nan"), float("nan"), float("nan")]
        sp.yaw = float("nan")
        sp.timestamp = ts
        self.setpoint_pub.publish(sp)

        self.offboard_counter += 1

    # ────────────── PX4 Komutları ──────────────

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

    # ────────────── Takeoff / Land ──────────────

    def do_takeoff(self):
        self.target = [0.0, 0.0, -self.takeoff_alt]
        print(f"  Setpoint gonderiyor ({self.offboard_counter} adet gonderildi)...")

        if self.offboard_counter < 20:
            print("  Henuz yeterli setpoint gonderilmedi, biraz bekleyin...")
            return

        # Offboard mode
        print("  OFFBOARD mod komutu gonderiliyor...")
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        # Kısa bekle, sonra arm
        import time
        time.sleep(0.5)

        print("  ARM komutu gonderiliyor...")
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        print(f"  [TAKEOFF] Hedef yukseklik: {self.takeoff_alt}m")

    def do_land(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        print("  [LAND] Inis komutu gonderildi")

    # ────────────── Klavye Döngüsü ──────────────

    def run_keyboard(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self._handle_key(key)
                    if key in ("\x1b", "\x03"):
                        break
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            if self.armed:
                self.do_land()

    def _handle_key(self, key):
        t = self.target
        if key == "t":
            self.do_takeoff()
        elif key == "l":
            self.do_land()
        elif key == "w":
            t[0] += self.step_xy
            print(f"  > Ileri  | x={t[0]:.1f} y={t[1]:.1f} z={t[2]:.1f}")
        elif key == "s":
            t[0] -= self.step_xy
            print(f"  > Geri   | x={t[0]:.1f} y={t[1]:.1f} z={t[2]:.1f}")
        elif key == "a":
            t[1] += self.step_xy
            print(f"  > Sol    | x={t[0]:.1f} y={t[1]:.1f} z={t[2]:.1f}")
        elif key == "d":
            t[1] -= self.step_xy
            print(f"  > Sag    | x={t[0]:.1f} y={t[1]:.1f} z={t[2]:.1f}")
        elif key == "q":
            t[2] -= self.step_z
            print(f"  > Yukari | x={t[0]:.1f} y={t[1]:.1f} z={t[2]:.1f}")
        elif key == "e":
            t[2] += self.step_z
            print(f"  > Asagi  | x={t[0]:.1f} y={t[1]:.1f} z={t[2]:.1f}")
        elif key == " ":
            self.target = list(self.pos)
            t = self.target
            print(f"  > HOVER  | x={t[0]:.1f} y={t[1]:.1f} z={t[2]:.1f}")


def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleop()

    # ROS spin ayrı thread'de — heartbeat hiç kesilmez
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.run_keyboard()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
