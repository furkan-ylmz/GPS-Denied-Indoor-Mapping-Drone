"""
Drone Teleop - Klavye ile drone kontrolu
ROS spin ayri thread'de calisir, heartbeat hic kesilmez.
QoS: PX4 uRTPS bridge uyumlu.
"""

import sys
import termios
import tty
import threading
import select
import time

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
+-------------------------------------------+
|         Drone Teleop Kontrolu             |
+-------------------------------------------+
|  t   : Arm + Offboard + Takeoff          |
|  w/s : Ileri (+x) / Geri (-x)  [0.5m]   |
|  a/d : Sol (+y) / Sag (-y)     [0.5m]   |
|  q/e : Yukari / Asagi          [0.3m]   |
|  l   : Land                              |
|  SPC : Hover (yerinde dur)               |
|  ESC : Cikis                             |
+-------------------------------------------+
  NOT: Once 't' ile arm+takeoff yapin!
"""


class DroneTeleop(Node):
    def __init__(self):
        super().__init__("drone_teleop")

        self.step_xy = 0.5
        self.step_z = 0.3
        self.takeoff_height = 1.5

        # PX4 icin publisher QoS - RELIABLE + VOLATILE
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # PX4 icin subscriber QoS - BEST_EFFORT
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", pub_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", pub_qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", pub_qos)

        self.pos_sub = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            self._pos_cb, sub_qos)
        self.status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status",
            self._status_cb, sub_qos)

        self.position = [0.0, 0.0, 0.0]
        self.target = [0.0, 0.0, -self.takeoff_height]
        self.armed = False
        self.nav_state = 0
        self.offboard_counter = 0

        # 10 Hz heartbeat (kararlilk icin)
        self.timer = self.create_timer(0.1, self._heartbeat)

        self.get_logger().info("Drone Teleop baslatildi")
        print(HELP_TEXT)

    def _pos_cb(self, msg):
        self.position = [msg.x, msg.y, msg.z]

    def _status_cb(self, msg):
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.nav_state = msg.nav_state

    def _heartbeat(self):
        ts = int(self.get_clock().now().nanoseconds / 1000)

        mode = OffboardControlMode()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = ts
        self.offboard_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.position = [float(self.target[0]), float(self.target[1]), float(self.target[2])]
        sp.yaw = 0.0
        sp.timestamp = ts
        self.setpoint_pub.publish(sp)

        self.offboard_counter += 1

    def _send_cmd(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.command = cmd
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def do_takeoff(self):
        self.target = [0.0, 0.0, -self.takeoff_height]
        print("  Setpoint gonderiyor (2 saniye bekleniyor)...")

        # PX4 offboard'a gecmeden once en az 1-2 sn setpoint istiyor
        time.sleep(2.0)

        # Offboard mode
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        time.sleep(0.5)

        # Arm
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        print(f"  [ARM + OFFBOARD] Takeoff -> {self.takeoff_height}m")

    def do_land(self):
        self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        print("  [LAND] Inis komutu gonderildi")

    def run_keyboard(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == "t":
                        self.do_takeoff()
                    elif key == "l":
                        self.do_land()
                    elif key == "w":
                        self.target[0] += self.step_xy
                        print(f"  > Ileri  | x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                    elif key == "s":
                        self.target[0] -= self.step_xy
                        print(f"  > Geri   | x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                    elif key == "a":
                        self.target[1] += self.step_xy
                        print(f"  > Sol    | x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                    elif key == "d":
                        self.target[1] -= self.step_xy
                        print(f"  > Sag    | x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                    elif key == "q":
                        self.target[2] -= self.step_z
                        print(f"  > Yukari | x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                    elif key == "e":
                        self.target[2] += self.step_z
                        print(f"  > Asagi  | x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                    elif key == " ":
                        self.target = list(self.position)
                        print(f"  > HOVER  | x={self.target[0]:.1f} y={self.target[1]:.1f} z={self.target[2]:.1f}")
                    elif key in ("\x1b", "\x03"):
                        break
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.do_land()


def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleop()

    # ROS spin ayri thread'de - heartbeat hic kesilmez
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_keyboard()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
