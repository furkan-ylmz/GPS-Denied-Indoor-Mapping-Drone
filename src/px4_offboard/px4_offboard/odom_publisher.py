"""
PX4 Odometry → ROS 2 nav_msgs/Odometry + TF Dönüştürücü
─────────────────────────────────────────────────────────
PX4'ün VehicleOdometry (NED/FRD) verisini ROS 2 standart
nav_msgs/Odometry (ENU/FLU) formatına çevirir.

Yayınlar:
  /drone/odom           → nav_msgs/Odometry (ENU frame)
  TF: odom → base_link  (odom frame)

Dönüşüm:
  NED → ENU:  x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
  FRD → FLU:  quaternion dönüşümü (Hamilton convention)

Kullanım:
  ros2 run px4_offboard odom_publisher
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# ── QoS: PX4 uXRCE-DDS ile uyumlu ──
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OdomPublisher(Node):
    """PX4 → ROS 2 odometry dönüştürücü."""

    def __init__(self):
        super().__init__("odom_publisher")

        # ── Subscriber: PX4 VehicleOdometry (v0 → suffix yok) ──
        self.sub_odom = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self._odom_cb,
            PX4_QOS,
        )

        # ── Publisher: ROS 2 Odometry ──
        self.pub_odom = self.create_publisher(Odometry, "/drone/odom", 10)

        # ── TF Broadcaster: odom → base_link ──
        self.tf_broadcaster = TransformBroadcaster(self)

        self._msg_count = 0
        self.get_logger().info("PX4 Odometry → ROS 2 dönüştürücü başlatıldı")

    def _odom_cb(self, msg: VehicleOdometry):
        """PX4 VehicleOdometry → nav_msgs/Odometry + TF dönüşümü."""
        now = self.get_clock().now().to_msg()

        # ── NED → ENU pozisyon dönüşümü ──
        # PX4 NED: x=North, y=East, z=Down
        # ROS ENU: x=East,  y=North, z=Up
        pos = msg.position  # [x_ned, y_ned, z_ned]
        x_enu = pos[1]       # East
        y_enu = pos[0]       # North
        z_enu = -pos[2]      # Up

        # ── NED/FRD → ENU/FLU quaternion dönüşümü ──
        # PX4 quaternion: body FRD → world NED  (w, x, y, z Hamilton)
        # ROS quaternion: body FLU → world ENU
        # Tam dönüşüm: q_enu_flu = q_ned2enu ⊗ q_ned_frd ⊗ q_frd2flu
        #   q_ned2enu = (0, 1/√2, 1/√2, 0)   [NED→ENU referans çerçeve rotasyonu]
        #   q_frd2flu = (0, 1, 0, 0)           [π rad x-ekseni etrafında rotasyon]
        q = msg.q  # [w, x, y, z]
        _s = 0.7071067811865476  # 1/√2
        qw = (q[0] + q[3]) * _s
        qx = (q[1] + q[2]) * _s
        qy = (q[1] - q[2]) * _s
        qz = (q[0] - q[3]) * _s

        # ── NED → ENU hız dönüşümü ──
        vel = msg.velocity  # [vx_ned, vy_ned, vz_ned]
        vx_enu = vel[1]
        vy_enu = vel[0]
        vz_enu = -vel[2]

        # Angular velocity: FRD → FLU (body frame dönüşüm, swap değil)
        ang = msg.angular_velocity  # [wx, wy, wz] body FRD
        wx_flu = ang[0]       # forward ekseni (aynı)
        wy_flu = -ang[1]      # right → left (negate)
        wz_flu = -ang[2]      # down → up (negate)

        # ── nav_msgs/Odometry mesajı ──
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = float(x_enu)
        odom.pose.pose.position.y = float(y_enu)
        odom.pose.pose.position.z = float(z_enu)

        # Orientation
        odom.pose.pose.orientation.w = float(qw)
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)

        # Covariance (PX4 variance → diagonal covariance)
        pv = msg.position_variance   # [var_x, var_y, var_z] NED
        ov = msg.orientation_variance  # [var_roll, var_pitch, var_yaw]
        odom.pose.covariance[0] = float(pv[1])   # x (East)
        odom.pose.covariance[7] = float(pv[0])   # y (North)
        odom.pose.covariance[14] = float(pv[2])  # z
        odom.pose.covariance[21] = float(ov[1])  # roll
        odom.pose.covariance[28] = float(ov[0])  # pitch
        odom.pose.covariance[35] = float(ov[2])  # yaw

        # Linear velocity
        odom.twist.twist.linear.x = float(vx_enu)
        odom.twist.twist.linear.y = float(vy_enu)
        odom.twist.twist.linear.z = float(vz_enu)

        # Angular velocity
        odom.twist.twist.angular.x = float(wx_flu)
        odom.twist.twist.angular.y = float(wy_flu)
        odom.twist.twist.angular.z = float(wz_flu)

        vv = msg.velocity_variance  # [var_vx, var_vy, var_vz] NED
        odom.twist.covariance[0] = float(vv[1])
        odom.twist.covariance[7] = float(vv[0])
        odom.twist.covariance[14] = float(vv[2])

        self.pub_odom.publish(odom)

        # ── TF: odom → base_link ──
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = float(x_enu)
        tf.transform.translation.y = float(y_enu)
        tf.transform.translation.z = float(z_enu)
        tf.transform.rotation.w = float(qw)
        tf.transform.rotation.x = float(qx)
        tf.transform.rotation.y = float(qy)
        tf.transform.rotation.z = float(qz)

        self.tf_broadcaster.sendTransform(tf)

        self._msg_count += 1
        if self._msg_count == 1:
            self.get_logger().info("İlk PX4 odometry alındı — yayın aktif")
        elif self._msg_count % 500 == 0:
            self.get_logger().info(
                f"Odom #{self._msg_count}: pos=({x_enu:.2f}, {y_enu:.2f}, {z_enu:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
