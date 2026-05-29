"""PX4 TF Broadcaster — VehicleOdometry'den odom->base_link TF yayınlar.

Koordinat Dönüşümü:
  PX4: NED frame (North-East-Down), FRD body (Forward-Right-Down)
  ROS: ENU frame (East-North-Up),   FLU body (Forward-Left-Up)

  Pozisyon: enu = (ned_y, ned_x, -ned_z)
  Quaternion: q_ros = q_NED_to_ENU ⊗ q_PX4 ⊗ q_FLU_to_FRD
    → q_ros = (s*(w+z), s*(x+y), s*(x-y), s*(w-z))  where s = √2/2
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math

SQRT2_2 = math.sqrt(2.0) / 2.0


def px4_to_ros_quaternion(q_px4):
    """PX4 NED/FRD quaternion → ROS ENU/FLU quaternion.

    Full derivation:
      q_ros = q_NED_to_ENU ⊗ q_PX4 ⊗ q_FLU_to_FRD
    where:
      q_NED_to_ENU = (0, √2/2, √2/2, 0)   [180° rotation about (1,1,0)/√2]
      q_FLU_to_FRD = (0, 1, 0, 0)          [180° rotation about x-axis]

    Simplified closed-form (s = √2/2):
      w_ros = s * (w + z)
      x_ros = s * (x + y)
      y_ros = s * (x - y)
      z_ros = s * (w - z)
    """
    w, x, y, z = q_px4[0], q_px4[1], q_px4[2], q_px4[3]
    return (
        SQRT2_2 * (w + z),
        SQRT2_2 * (x + y),
        SQRT2_2 * (x - y),
        SQRT2_2 * (w - z),
    )


class PX4TfBroadcaster(Node):
    def __init__(self):
        super().__init__('px4_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/drone/odom', 10)
        self.msg_count = 0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile,
        )

        self.get_logger().info('TF Broadcaster başlatıldı — /fmu/out/vehicle_odometry dinleniyor')

    def odom_callback(self, msg):
        # Skip if data has NaN
        if any(math.isnan(v) for v in msg.position):
            return
        if any(math.isnan(v) for v in msg.q):
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Position: NED → ENU
        t.transform.translation.x = float(msg.position[1])   # NED-East  → ENU-x
        t.transform.translation.y = float(msg.position[0])   # NED-North → ENU-y
        t.transform.translation.z = -float(msg.position[2])  # NED-Down  → ENU-Up

        # Quaternion: NED/FRD → ENU/FLU
        q_ros = px4_to_ros_quaternion(msg.q)
        t.transform.rotation.w = float(q_ros[0])
        t.transform.rotation.x = float(q_ros[1])
        t.transform.rotation.y = float(q_ros[2])
        t.transform.rotation.z = float(q_ros[3])

        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = t.child_frame_id
        odom.pose.pose.position.x = t.transform.translation.x
        odom.pose.pose.position.y = t.transform.translation.y
        odom.pose.pose.position.z = t.transform.translation.z
        odom.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(odom)

        self.msg_count += 1
        if self.msg_count == 1:
            self.get_logger().info('✅ İlk odom->base_link TF ve /drone/odom yayınlandı!')


def main(args=None):
    rclpy.init(args=args)
    node = PX4TfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
