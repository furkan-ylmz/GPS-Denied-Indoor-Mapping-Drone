import json
import math
import os
from typing import Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, String
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _distance_xy(a: Dict[str, float], b: Dict[str, float]) -> float:
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def _default_map_json_path() -> str:
    package_root = os.path.abspath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir)
    )
    docs_dir = os.path.join(package_root, "docs")
    if os.path.isdir(docs_dir):
        return os.path.join(docs_dir, "door_labels.json")

    try:
        pkg_share = get_package_share_directory("door_ocr")
        return os.path.join(pkg_share, "docs", "door_labels.json")
    except Exception:
        return os.path.expanduser("~/door_labels.json")


def _resolve_map_json_path(path: str) -> str:
    expanded = os.path.expanduser(path.strip())
    if not expanded:
        return _default_map_json_path()
    if os.path.isabs(expanded):
        return expanded

    package_root = os.path.abspath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir)
    )
    if os.path.isdir(os.path.join(package_root, "docs")):
        return os.path.join(package_root, expanded)

    try:
        pkg_share = get_package_share_directory("door_ocr")
        return os.path.join(pkg_share, expanded)
    except Exception:
        return os.path.abspath(expanded)


class DoorSemanticMapper(Node):
    """Attach confirmed OCR labels to approximate odom positions.

    This node is intentionally conservative: it consumes only confirmed OCR labels,
    merges repeated observations, publishes RViz text markers, and persists a JSON
    sidecar file. The geometry map remains untouched.
    """

    def __init__(self) -> None:
        super().__init__("door_semantic_mapper")

        self.declare_parameter("confirmed_label_topic", "/ocr/door_label_confirmed")
        self.declare_parameter("confirmed_valid_topic", "/ocr/door_label_confirmed_valid")
        self.declare_parameter("odom_topic", "/drone/odom")
        self.declare_parameter("reset_topic", "/ocr/door_map_reset")
        self.declare_parameter("reset_confirmation_topic", "/ocr/reset_confirmation")
        self.declare_parameter("marker_topic", "/ocr/door_label_markers")
        self.declare_parameter("map_json_path", _default_map_json_path())
        self.declare_parameter("marker_frame_id", "odom")
        self.declare_parameter("label_distance_ahead_m", 1.5)
        self.declare_parameter("use_fixed_marker_z", True)
        self.declare_parameter("fixed_marker_z", 1.4)
        self.declare_parameter("marker_z_offset", 0.0)
        self.declare_parameter("merge_radius_m", 1.0)
        self.declare_parameter("allow_duplicate_labels", False)
        self.declare_parameter("min_accept_interval_sec", 1.0)
        self.declare_parameter("stale_odom_timeout_sec", 2.0)
        self.declare_parameter("marker_publish_period_sec", 0.5)
        self.declare_parameter("text_scale", 0.35)
        self.declare_parameter("anchor_scale", 0.18)
        self.declare_parameter("promote_alternative_labels", True)
        self.declare_parameter("label_promotion_min_observations", 3)
        self.declare_parameter("label_promotion_margin", 2)

        self.confirmed_label_topic = str(self.get_parameter("confirmed_label_topic").value)
        self.confirmed_valid_topic = str(self.get_parameter("confirmed_valid_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.reset_topic = str(self.get_parameter("reset_topic").value)
        self.reset_confirmation_topic = str(
            self.get_parameter("reset_confirmation_topic").value
        )
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.map_json_path = _resolve_map_json_path(str(self.get_parameter("map_json_path").value))
        self.marker_frame_id = str(self.get_parameter("marker_frame_id").value).strip() or "odom"
        self.label_distance_ahead_m = float(self.get_parameter("label_distance_ahead_m").value)
        self.use_fixed_marker_z = bool(self.get_parameter("use_fixed_marker_z").value)
        self.fixed_marker_z = float(self.get_parameter("fixed_marker_z").value)
        self.marker_z_offset = float(self.get_parameter("marker_z_offset").value)
        self.merge_radius_m = float(self.get_parameter("merge_radius_m").value)
        self.allow_duplicate_labels = bool(self.get_parameter("allow_duplicate_labels").value)
        self.min_accept_interval_sec = float(self.get_parameter("min_accept_interval_sec").value)
        self.stale_odom_timeout_sec = float(self.get_parameter("stale_odom_timeout_sec").value)
        self.text_scale = float(self.get_parameter("text_scale").value)
        self.anchor_scale = float(self.get_parameter("anchor_scale").value)
        self.promote_alternative_labels = bool(
            self.get_parameter("promote_alternative_labels").value
        )
        self.label_promotion_min_observations = max(
            1, int(self.get_parameter("label_promotion_min_observations").value)
        )
        self.label_promotion_margin = max(
            0, int(self.get_parameter("label_promotion_margin").value)
        )

        marker_period = max(0.1, float(self.get_parameter("marker_publish_period_sec").value))

        self.last_odom: Optional[Odometry] = None
        self.last_odom_time = self.get_clock().now()
        self.current_label = ""
        self.current_valid = False
        self.last_accept_time_sec = -1.0
        self.loaded_entries_changed = False
        self.entries: List[Dict] = self._load_entries()
        if self.loaded_entries_changed:
            self._save_entries()

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.reset_confirmation_pub = self.create_publisher(
            Empty, self.reset_confirmation_topic, 10
        )
        self.create_subscription(String, self.confirmed_label_topic, self._label_cb, 10)
        self.create_subscription(Bool, self.confirmed_valid_topic, self._valid_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(Bool, self.reset_topic, self._reset_cb, 10)
        self.create_timer(marker_period, self._publish_markers)

        self.get_logger().info(
            "door_semantic_mapper ready | "
            f"label={self.confirmed_label_topic} valid={self.confirmed_valid_topic} "
            f"odom={self.odom_topic} markers={self.marker_topic} json={self.map_json_path}"
        )
        if self.entries:
            self.get_logger().info(f"loaded {len(self.entries)} persisted door labels")

    def _label_cb(self, msg: String) -> None:
        self.current_label = msg.data.strip()
        self._try_accept_current_label()

    def _valid_cb(self, msg: Bool) -> None:
        self.current_valid = bool(msg.data)
        self._try_accept_current_label()

    def _odom_cb(self, msg: Odometry) -> None:
        self.last_odom = msg
        self.last_odom_time = self.get_clock().now()
        self._try_accept_current_label()

    def _reset_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.entries = []
        self._save_entries()
        self._publish_delete_all()
        self.get_logger().warn("door semantic map reset")

    def _try_accept_current_label(self) -> None:
        if not self.current_valid or not self.current_label or self.last_odom is None:
            return

        now = self.get_clock().now()
        if (now - self.last_odom_time).nanoseconds / 1e9 > self.stale_odom_timeout_sec:
            return

        now_sec = now.nanoseconds / 1e9
        if (
            self.last_accept_time_sec > 0.0
            and now_sec - self.last_accept_time_sec < self.min_accept_interval_sec
        ):
            return

        position, yaw, frame_id = self._estimate_label_pose()
        accepted = self._upsert_entry(self.current_label, position, yaw, frame_id, now_sec)
        if accepted:
            self.last_accept_time_sec = now_sec
            self._save_entries()
            self._publish_markers()
            self._reset_ocr_confirmation()

    def _reset_ocr_confirmation(self) -> None:
        self.current_label = ""
        self.current_valid = False
        self.reset_confirmation_pub.publish(Empty())

    def _estimate_label_pose(self) -> Tuple[Dict[str, float], float, str]:
        assert self.last_odom is not None
        pose = self.last_odom.pose.pose
        yaw = _yaw_from_quaternion(pose.orientation)
        x = pose.position.x + math.cos(yaw) * self.label_distance_ahead_m
        y = pose.position.y + math.sin(yaw) * self.label_distance_ahead_m
        if self.use_fixed_marker_z:
            z = self.fixed_marker_z
        else:
            z = pose.position.z + self.marker_z_offset
        frame_id = self.marker_frame_id or self.last_odom.header.frame_id or "odom"
        return {"x": x, "y": y, "z": z}, yaw, frame_id

    def _upsert_entry(
        self,
        label: str,
        position: Dict[str, float],
        yaw: float,
        frame_id: str,
        now_sec: float,
    ) -> bool:
        nearest_same = None
        nearest_same_dist = float("inf")
        nearest_any = None
        nearest_any_dist = float("inf")

        for entry in self.entries:
            dist = _distance_xy(entry["position"], position)
            if dist < nearest_any_dist:
                nearest_any = entry
                nearest_any_dist = dist
            if entry["label"] == label and dist < nearest_same_dist:
                nearest_same = entry
                nearest_same_dist = dist

        if nearest_same is not None:
            if nearest_same_dist <= self.merge_radius_m:
                self._merge_observation(nearest_same, position, yaw, now_sec)
                return True
            if not self.allow_duplicate_labels:
                return False

        if nearest_any is not None and nearest_any_dist <= self.merge_radius_m:
            if nearest_any["label"] != label:
                alternatives = nearest_any.setdefault("alternative_labels", {})
                alternatives[label] = int(alternatives.get(label, 0)) + 1
                nearest_any["last_seen_sec"] = now_sec
                if not self._maybe_promote_alternative_label(nearest_any, label):
                    self.get_logger().warn(
                        f"label conflict near existing door: kept {nearest_any['label']}, "
                        f"ignored {label}"
                    )
                return True

        self.entries.append(
            {
                "door_id": f"door_{len(self.entries) + 1:03d}",
                "label": label,
                "position": {
                    "x": float(position["x"]),
                    "y": float(position["y"]),
                    "z": float(position["z"]),
                },
                "yaw": float(yaw),
                "frame_id": frame_id,
                "observations": 1,
                "first_seen_sec": now_sec,
                "last_seen_sec": now_sec,
                "source": "ocr_confirmed",
            }
        )
        self.get_logger().info(
            f"mapped door label {label} at "
            f"x={position['x']:.2f} y={position['y']:.2f} z={position['z']:.2f}"
        )
        return True

    def _merge_observation(
        self, entry: Dict, position: Dict[str, float], yaw: float, now_sec: float
    ) -> None:
        n = max(1, int(entry.get("observations", 1)))
        weight_old = min(n, 12)
        weight_new = 1
        denom = weight_old + weight_new
        entry["position"]["x"] = (
            float(entry["position"]["x"]) * weight_old + float(position["x"]) * weight_new
        ) / denom
        entry["position"]["y"] = (
            float(entry["position"]["y"]) * weight_old + float(position["y"]) * weight_new
        ) / denom
        entry["position"]["z"] = (
            float(entry["position"]["z"]) * weight_old + float(position["z"]) * weight_new
        ) / denom
        entry["yaw"] = (float(entry.get("yaw", yaw)) * weight_old + float(yaw)) / denom
        entry["observations"] = n + 1
        entry["last_seen_sec"] = now_sec

    def _load_entries(self) -> List[Dict]:
        if not os.path.exists(self.map_json_path):
            return []
        try:
            with open(self.map_json_path, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            frame_hint = ""
            if isinstance(payload, dict):
                entries = payload.get("doors", [])
                frame_hint = str(payload.get("frame_hint", "")).strip()
            else:
                entries = payload
            if not isinstance(entries, list):
                return []
            odom_entries = []
            skipped = 0
            for entry in entries:
                if not isinstance(entry, dict):
                    skipped += 1
                    continue
                frame_id = str(entry.get("frame_id") or frame_hint or self.marker_frame_id).strip()
                if frame_id != self.marker_frame_id:
                    skipped += 1
                    continue
                entry["frame_id"] = self.marker_frame_id
                if self._maybe_promote_alternative_label(entry):
                    self.loaded_entries_changed = True
                odom_entries.append(entry)
            if skipped:
                self.get_logger().warn(
                    f"ignored {skipped} persisted door labels outside {self.marker_frame_id} frame"
                )
            return odom_entries
        except Exception as exc:
            self.get_logger().warn(f"could not load door label map: {exc}")
            return []

    def _maybe_promote_alternative_label(
        self, entry: Dict, candidate_label: Optional[str] = None
    ) -> bool:
        if not self.promote_alternative_labels:
            return False
        alternatives = entry.get("alternative_labels", {})
        if not isinstance(alternatives, dict) or not alternatives:
            return False

        labels = [candidate_label] if candidate_label else list(alternatives.keys())
        best_label = ""
        best_count = -1
        for label in labels:
            if not label or label == entry.get("label"):
                continue
            count = int(alternatives.get(label, 0))
            if count > best_count:
                best_label = label
                best_count = count

        if not best_label:
            return False

        current_label = str(entry.get("label", ""))
        current_count = max(1, int(entry.get("observations", 1)))
        if best_count < self.label_promotion_min_observations:
            return False
        if best_count < current_count + self.label_promotion_margin:
            return False

        alternatives.pop(best_label, None)
        if current_label:
            alternatives[current_label] = max(
                int(alternatives.get(current_label, 0)), current_count
            )
        if alternatives:
            entry["alternative_labels"] = alternatives
        else:
            entry.pop("alternative_labels", None)

        entry["label"] = best_label
        entry["observations"] = best_count
        entry["source"] = "ocr_confirmed_corrected"
        self.get_logger().warn(
            f"promoted door label near existing door: {current_label} -> {best_label} "
            f"(votes={best_count}, previous={current_count})"
        )
        return True

    def _save_entries(self) -> None:
        payload = {
            "format": "door_semantic_map_v1",
            "frame_hint": self.marker_frame_id,
            "doors": self.entries,
        }
        try:
            os.makedirs(os.path.dirname(self.map_json_path), exist_ok=True)
            tmp_path = self.map_json_path + ".tmp"
            with open(tmp_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, ensure_ascii=False)
                handle.write("\n")
            os.replace(tmp_path, self.map_json_path)
        except Exception as exc:
            self.get_logger().warn(f"could not save door label map: {exc}")

    def _publish_delete_all(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.marker_frame_id
        marker.header.stamp = Time().to_msg()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def _publish_markers(self) -> None:
        stamp = Time().to_msg()
        delete_all = Marker()
        delete_all.header.frame_id = self.marker_frame_id
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        markers = [delete_all]
        for index, entry in enumerate(self.entries):
            position = entry["position"]
            frame_id = self.marker_frame_id

            text = Marker()
            text.header.frame_id = frame_id
            text.header.stamp = stamp
            text.ns = "door_labels"
            text.id = index * 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(position["x"])
            text.pose.position.y = float(position["y"])
            text.pose.position.z = float(position["z"]) + 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = self.text_scale
            text.color.r = 0.0
            text.color.g = 1.0
            text.color.b = 0.0
            text.color.a = 1.0
            text.text = str(entry["label"])
            markers.append(text)

            anchor = Marker()
            anchor.header.frame_id = frame_id
            anchor.header.stamp = stamp
            anchor.ns = "door_anchor"
            anchor.id = index * 2 + 1
            anchor.type = Marker.SPHERE
            anchor.action = Marker.ADD
            anchor.pose.position.x = float(position["x"])
            anchor.pose.position.y = float(position["y"])
            anchor.pose.position.z = float(position["z"])
            anchor.pose.orientation.w = 1.0
            anchor.scale.x = self.anchor_scale
            anchor.scale.y = self.anchor_scale
            anchor.scale.z = self.anchor_scale
            anchor.color.r = 1.0
            anchor.color.g = 0.85
            anchor.color.b = 0.0
            anchor.color.a = 0.95
            markers.append(anchor)

        self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DoorSemanticMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
