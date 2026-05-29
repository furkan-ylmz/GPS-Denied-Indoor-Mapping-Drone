"""Door OCR Node with pluggable backend (rapidocr / paddle)."""

import json
import re
from collections import deque
from typing import Dict, List, Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, String


class DoorOcrNode(Node):
    def __init__(self):
        super().__init__("door_ocr_node")

        self.declare_parameter("image_topic", "/camera")
        self.declare_parameter("label_topic", "/ocr/door_label")
        self.declare_parameter("valid_topic", "/ocr/door_label_valid")
        self.declare_parameter("labels_all_topic", "/ocr/door_labels_all")
        self.declare_parameter("detections_topic", "/ocr/door_label_detections")
        self.declare_parameter("confirmed_label_topic", "/ocr/door_label_confirmed")
        self.declare_parameter("confirmed_valid_topic", "/ocr/door_label_confirmed_valid")
        self.declare_parameter("needs_closer_view_topic", "/ocr/needs_closer_view")
        self.declare_parameter("reset_confirmation_topic", "/ocr/reset_confirmation")
        self.declare_parameter("active_target_topic", "/ocr/active_target")
        self.declare_parameter("target_roi_topic", "/ocr/target_roi")
        self.declare_parameter("debug_image_topic", "/ocr/debug_image")
        self.declare_parameter("crop_debug_image_topic", "/ocr/crop_debug_image")
        self.declare_parameter("legacy_debug_image_topic", "/ocr/raw")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("process_every_n", 3)
        self.declare_parameter("max_labels_per_frame", 5)
        self.declare_parameter("label_regex", r"^[A-Z0-9][A-Z0-9-]{1,20}$")
        self.declare_parameter("language", "en")
        self.declare_parameter("use_gpu", False)
        self.declare_parameter("backend", "rapidocr")
        self.declare_parameter("enable_crop_retry", True)
        self.declare_parameter("crop_retry_scale", 2.0)
        self.declare_parameter("crop_retry_padding_px", 28)
        self.declare_parameter("crop_retry_accept_threshold", 0.55)
        self.declare_parameter("enable_crop_retry_variants", True)
        self.declare_parameter("crop_retry_mode", "sharp_only")
        self.declare_parameter("crop_retry_scale_mode", "dynamic")
        self.declare_parameter("crop_retry_scale_candidates", "2.0,2.5,3.0")
        self.declare_parameter("crop_retry_target_height_px", 105)
        self.declare_parameter("crop_retry_max_scale", 3.0)
        self.declare_parameter("crop_debug_target_height_px", 180)
        self.declare_parameter("crop_retry_variant_min_score", 0.82)
        self.declare_parameter("crop_retry_refine_max_candidates", 1)
        self.declare_parameter("crop_retry_fallback_max_candidates", 2)
        self.declare_parameter("enable_roi_retry", True)
        self.declare_parameter("roi_retry_x_min", 0.15)
        self.declare_parameter("roi_retry_x_max", 0.85)
        self.declare_parameter("roi_retry_y_min", 0.20)
        self.declare_parameter("roi_retry_y_max", 0.62)
        self.declare_parameter("enable_sign_detection_retry", True)
        self.declare_parameter("sign_retry_min_area_px", 18)
        self.declare_parameter("sign_retry_max_area_ratio", 0.018)
        self.declare_parameter("sign_retry_min_aspect", 2.0)
        self.declare_parameter("sign_retry_max_aspect", 12.0)
        self.declare_parameter("sign_retry_padding_ratio", 0.90)
        self.declare_parameter("sign_retry_max_candidates", 2)
        self.declare_parameter("vote_window_size", 10)
        self.declare_parameter("vote_min_observations", 6)
        self.declare_parameter("vote_min_char_ratio", 0.60)
        self.declare_parameter("vote_min_char_margin", 0.20)
        self.declare_parameter("confirmed_change_min_observations", 8)
        self.declare_parameter("confirmed_change_min_char_ratio", 0.75)
        self.declare_parameter("confirmed_change_min_char_margin", 0.25)
        self.declare_parameter("ambiguous_char_margin", 0.35)
        self.declare_parameter("prefer_letter_suffix", False)
        self.declare_parameter("letter_suffix_digit_count", 3)
        self.declare_parameter("enable_confirmation_guard", True)
        self.declare_parameter("confirm_min_text_height_px", 8.0)
        self.declare_parameter("confirm_min_ambiguous_text_height_px", 18.0)
        self.declare_parameter("target_ambiguity_ratio", 0.70)
        self.declare_parameter("enable_target_gate", True)
        self.declare_parameter("target_gate_half_width", 0.20)
        self.declare_parameter("target_gate_half_height", 0.34)
        self.declare_parameter("guard_on_multiple_signs", True)
        self.declare_parameter("guard_confusable_label_changes", True)
        self.declare_parameter("enable_target_door_priority", True)
        self.declare_parameter("target_center_x", 0.50)
        self.declare_parameter("target_center_y", 0.47)
        self.declare_parameter("target_center_weight", 0.65)
        self.declare_parameter("target_size_weight", 0.25)

        image_topic = str(self.get_parameter("image_topic").value)
        label_topic = str(self.get_parameter("label_topic").value)
        valid_topic = str(self.get_parameter("valid_topic").value)
        labels_all_topic = str(self.get_parameter("labels_all_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        confirmed_label_topic = str(self.get_parameter("confirmed_label_topic").value)
        confirmed_valid_topic = str(self.get_parameter("confirmed_valid_topic").value)
        needs_closer_view_topic = str(
            self.get_parameter("needs_closer_view_topic").value
        )
        reset_confirmation_topic = str(
            self.get_parameter("reset_confirmation_topic").value
        )
        active_target_topic = str(self.get_parameter("active_target_topic").value)
        target_roi_topic = str(self.get_parameter("target_roi_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.crop_debug_image_topic = str(
            self.get_parameter("crop_debug_image_topic").value
        )
        self.legacy_debug_image_topic = str(
            self.get_parameter("legacy_debug_image_topic").value
        )
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.conf_threshold = float(self.get_parameter("confidence_threshold").value)
        self.process_every_n = max(1, int(self.get_parameter("process_every_n").value))
        self.max_labels_per_frame = max(1, int(self.get_parameter("max_labels_per_frame").value))
        self.label_re = re.compile(str(self.get_parameter("label_regex").value))
        language = str(self.get_parameter("language").value)
        use_gpu = bool(self.get_parameter("use_gpu").value)
        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.enable_crop_retry = bool(self.get_parameter("enable_crop_retry").value)
        self.crop_retry_scale = max(1.0, float(self.get_parameter("crop_retry_scale").value))
        self.crop_retry_padding_px = max(
            0, int(self.get_parameter("crop_retry_padding_px").value)
        )
        self.crop_retry_accept_threshold = float(
            self.get_parameter("crop_retry_accept_threshold").value
        )
        self.enable_crop_retry_variants = bool(
            self.get_parameter("enable_crop_retry_variants").value
        )
        self.crop_retry_mode = str(
            self.get_parameter("crop_retry_mode").value
        ).strip().lower()
        self.crop_retry_scale_mode = str(
            self.get_parameter("crop_retry_scale_mode").value
        ).strip().lower()
        self.crop_retry_target_height_px = max(
            32, int(self.get_parameter("crop_retry_target_height_px").value)
        )
        self.crop_retry_max_scale = max(
            1.0, float(self.get_parameter("crop_retry_max_scale").value)
        )
        self.crop_retry_scale_candidates = self._parse_scale_candidates(
            self.get_parameter("crop_retry_scale_candidates").value,
            self.crop_retry_scale,
            self.crop_retry_max_scale,
        )
        self.crop_debug_target_height_px = max(
            64, int(self.get_parameter("crop_debug_target_height_px").value)
        )
        self.crop_retry_variant_min_score = float(
            self.get_parameter("crop_retry_variant_min_score").value
        )
        self.crop_retry_refine_max_candidates = max(
            1, int(self.get_parameter("crop_retry_refine_max_candidates").value)
        )
        self.crop_retry_fallback_max_candidates = max(
            1, int(self.get_parameter("crop_retry_fallback_max_candidates").value)
        )
        self.enable_roi_retry = bool(self.get_parameter("enable_roi_retry").value)
        self.roi_retry_x_min = float(self.get_parameter("roi_retry_x_min").value)
        self.roi_retry_x_max = float(self.get_parameter("roi_retry_x_max").value)
        self.roi_retry_y_min = float(self.get_parameter("roi_retry_y_min").value)
        self.roi_retry_y_max = float(self.get_parameter("roi_retry_y_max").value)
        self.enable_sign_detection_retry = bool(
            self.get_parameter("enable_sign_detection_retry").value
        )
        self.sign_retry_min_area_px = max(
            1, int(self.get_parameter("sign_retry_min_area_px").value)
        )
        self.sign_retry_max_area_ratio = float(
            self.get_parameter("sign_retry_max_area_ratio").value
        )
        self.sign_retry_min_aspect = float(
            self.get_parameter("sign_retry_min_aspect").value
        )
        self.sign_retry_max_aspect = float(
            self.get_parameter("sign_retry_max_aspect").value
        )
        self.sign_retry_padding_ratio = float(
            self.get_parameter("sign_retry_padding_ratio").value
        )
        self.sign_retry_max_candidates = max(
            1, int(self.get_parameter("sign_retry_max_candidates").value)
        )
        self.vote_window_size = max(1, int(self.get_parameter("vote_window_size").value))
        self.vote_min_observations = max(
            1, int(self.get_parameter("vote_min_observations").value)
        )
        self.vote_min_char_ratio = float(self.get_parameter("vote_min_char_ratio").value)
        self.vote_min_char_margin = float(self.get_parameter("vote_min_char_margin").value)
        self.confirmed_change_min_observations = max(
            1, int(self.get_parameter("confirmed_change_min_observations").value)
        )
        self.confirmed_change_min_char_ratio = float(
            self.get_parameter("confirmed_change_min_char_ratio").value
        )
        self.confirmed_change_min_char_margin = float(
            self.get_parameter("confirmed_change_min_char_margin").value
        )
        self.ambiguous_char_margin = float(
            self.get_parameter("ambiguous_char_margin").value
        )
        self.prefer_letter_suffix = bool(
            self.get_parameter("prefer_letter_suffix").value
        )
        self.letter_suffix_digit_count = max(
            1, int(self.get_parameter("letter_suffix_digit_count").value)
        )
        self.enable_confirmation_guard = bool(
            self.get_parameter("enable_confirmation_guard").value
        )
        self.confirm_min_text_height_px = float(
            self.get_parameter("confirm_min_text_height_px").value
        )
        self.confirm_min_ambiguous_text_height_px = float(
            self.get_parameter("confirm_min_ambiguous_text_height_px").value
        )
        self.target_ambiguity_ratio = float(
            self.get_parameter("target_ambiguity_ratio").value
        )
        self.enable_target_gate = bool(
            self.get_parameter("enable_target_gate").value
        )
        self.target_gate_half_width = float(
            self.get_parameter("target_gate_half_width").value
        )
        self.target_gate_half_height = float(
            self.get_parameter("target_gate_half_height").value
        )
        self.guard_on_multiple_signs = bool(
            self.get_parameter("guard_on_multiple_signs").value
        )
        self.guard_confusable_label_changes = bool(
            self.get_parameter("guard_confusable_label_changes").value
        )
        self.enable_target_door_priority = bool(
            self.get_parameter("enable_target_door_priority").value
        )
        self.target_center_x = float(self.get_parameter("target_center_x").value)
        self.target_center_y = float(self.get_parameter("target_center_y").value)
        self.target_center_weight = float(
            self.get_parameter("target_center_weight").value
        )
        self.target_size_weight = float(self.get_parameter("target_size_weight").value)

        self.bridge = CvBridge()
        self.seq = 0
        self.last_published_label = ""
        self.last_published_all = ""
        self.last_logged_confirmed = ""
        self.last_logged_needs_closer = False
        self.last_logged_guard_reason = ""
        self.confirmed_label = ""
        self.needs_closer_view = False
        self.confirmation_guard_reason = ""
        self.active_target_id = ""
        self.target_roi: Optional[Tuple[float, float, float, float]] = None
        self.vote_window = deque(maxlen=self.vote_window_size)

        self.label_pub = self.create_publisher(String, label_topic, 10)
        self.valid_pub = self.create_publisher(Bool, valid_topic, 10)
        self.labels_all_pub = self.create_publisher(String, labels_all_topic, 10)
        self.detections_pub = self.create_publisher(String, detections_topic, 10)
        self.confirmed_label_pub = self.create_publisher(
            String, confirmed_label_topic, 10
        )
        self.confirmed_valid_pub = self.create_publisher(
            Bool, confirmed_valid_topic, 10
        )
        self.needs_closer_view_pub = self.create_publisher(
            Bool, needs_closer_view_topic, 10
        )
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.crop_debug_pub = self.create_publisher(
            Image, self.crop_debug_image_topic, 10
        )
        self.legacy_debug_pub = None
        if (
            self.legacy_debug_image_topic
            and self.legacy_debug_image_topic != self.debug_image_topic
        ):
            self.legacy_debug_pub = self.create_publisher(
                Image, self.legacy_debug_image_topic, 10
            )

        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.create_subscription(
            Empty, reset_confirmation_topic, self._reset_confirmation_cb, 10
        )
        self.create_subscription(
            String, active_target_topic, self._active_target_cb, 10
        )
        self.create_subscription(String, target_roi_topic, self._target_roi_cb, 10)

        self.ocr = self._create_ocr(language, use_gpu)

        debug_topics = self.debug_image_topic
        if self.legacy_debug_pub is not None:
            debug_topics = (
                f"{self.debug_image_topic}, {self.legacy_debug_image_topic}"
            )
        self.get_logger().info(
            f"door_ocr_node ready | backend={self.backend} | image={image_topic} | "
            f"debug_topics={debug_topics}, {self.crop_debug_image_topic} | "
            f"reset_topic={reset_confirmation_topic} | "
            f"active_target_topic={active_target_topic} | "
            f"target_roi_topic={target_roi_topic} | "
            f"process_every_n={self.process_every_n} | "
            f"crop_retry={self.enable_crop_retry} "
            f"scale_mode={self.crop_retry_scale_mode}:{self.crop_retry_scale_candidates} | "
            f"vote_window={self.vote_window_size} | "
            f"ambiguous_margin={self.ambiguous_char_margin}"
        )

    def _create_ocr(self, language: str, use_gpu: bool):
        if self.backend in ("rapidocr", "auto"):
            try:
                from rapidocr_onnxruntime import RapidOCR

                self.backend = "rapidocr"
                self.get_logger().info("Using RapidOCR backend")
                return RapidOCR()
            except Exception as exc:
                if self.backend == "rapidocr":
                    raise RuntimeError(f"RapidOCR init failed: {exc}")
                self.get_logger().warn(f"RapidOCR unavailable, fallback to paddle: {exc}")

        if self.backend in ("paddle", "auto"):
            try:
                from paddleocr import PaddleOCR
            except Exception as exc:
                raise RuntimeError(f"PaddleOCR import failed: {exc}")

            candidates = [
                {"use_angle_cls": False, "lang": language, "use_gpu": use_gpu, "show_log": False},
                {"use_angle_cls": False, "lang": language, "use_gpu": use_gpu},
                {"use_angle_cls": False, "lang": language, "show_log": False},
                {"use_angle_cls": False, "lang": language},
                {"lang": language},
                {},
            ]
            last_error = None
            for kwargs in candidates:
                try:
                    self.backend = "paddle"
                    ocr = PaddleOCR(**kwargs)
                    self.get_logger().info(f"PaddleOCR initialized with args: {kwargs}")
                    return ocr
                except Exception as exc:
                    last_error = exc
            raise RuntimeError(f"PaddleOCR init failed: {last_error}")

        raise RuntimeError(f"Unknown OCR backend: {self.backend}")

    def _run_ocr(self, bgr: "cv2.Mat"):
        if self.backend == "rapidocr":
            ocr_result, _ = self.ocr(bgr)
            return ocr_result

        try:
            return self.ocr.ocr(bgr, cls=False)
        except TypeError:
            return self.ocr.ocr(bgr)
        except Exception as exc:
            if "unexpected keyword argument 'cls'" in str(exc):
                return self.ocr.ocr(bgr)
            raise

    @staticmethod
    def _normalize_label(text: str) -> str:
        # Keep hyphens so published labels can match door sign format (e.g. EG-001A).
        cleaned = text.strip().upper()
        cleaned = cleaned.replace("—", "-").replace("–", "-").replace("_", "-")
        cleaned = re.sub(r"\s+", "", cleaned)
        cleaned = re.sub(r"[^A-Z0-9-]", "", cleaned)
        cleaned = re.sub(r"-{2,}", "-", cleaned)
        return cleaned

    def _apply_format_hints(self, normalized: str) -> str:
        if not self.prefer_letter_suffix or not normalized:
            return normalized

        if "-" in normalized:
            prefix, body = normalized.split("-", 1)
        else:
            match = re.match(r"^([A-Z]+)([A-Z0-9]+)$", normalized)
            if match is None:
                return normalized
            prefix, body = match.group(1), match.group(2)

        digit_count = self.letter_suffix_digit_count
        match = re.match(rf"^([0-9]{{{digit_count}}})([68])$", body)
        if match is None:
            return normalized

        return self._format_label(prefix, f"{match.group(1)}B")

    @staticmethod
    def _canonical_label(text: str) -> str:
        # Canonical key for deduplication: ignore hyphen differences.
        return text.replace("-", "")

    def _same_label(self, left: str, right: str) -> bool:
        return self._canonical_label(left) == self._canonical_label(right)

    def _split_label(self, label: str) -> Optional[Tuple[str, str]]:
        normalized = self._normalize_label(label)
        if not normalized:
            return None

        if "-" in normalized:
            prefix, body = normalized.split("-", 1)
        else:
            match = re.match(r"^([A-Z]+)([A-Z0-9]+)$", normalized)
            if match is None:
                return None
            prefix, body = match.group(1), match.group(2)

        if not prefix or not body:
            return None
        return prefix, body

    @staticmethod
    def _format_label(prefix: str, body: str) -> str:
        return f"{prefix}-{body}"

    @staticmethod
    def _candidate_weight(score: float, rank: int) -> float:
        score = min(max(float(score), 0.0), 1.0)
        rank_weight = max(0.35, 1.0 - 0.30 * rank)
        return max(0.10, score) * rank_weight

    @staticmethod
    def _choose_char(votes: Dict[str, float]) -> Tuple[str, float, float]:
        if not votes:
            return "", 0.0, 0.0
        total = sum(votes.values())
        ordered = sorted(votes.items(), key=lambda item: item[1], reverse=True)
        best_char, best_weight = ordered[0]
        second_weight = ordered[1][1] if len(ordered) > 1 else 0.0
        ratio = best_weight / total if total > 0.0 else 0.0
        margin = (best_weight - second_weight) / total if total > 0.0 else 0.0
        return best_char, ratio, margin

    @staticmethod
    def _is_ambiguous_char_pair(left: str, right: str) -> bool:
        if not left or not right:
            return False
        ambiguous_groups = (
            frozenset(("B", "8", "6")),
            frozenset(("E", "F")),
            frozenset(("G", "6")),
            frozenset(("O", "0")),
            frozenset(("I", "1", "L")),
            frozenset(("S", "5")),
            frozenset(("Z", "2")),
        )
        return any(left in group and right in group for group in ambiguous_groups)

    def _label_needs_ambiguous_variants(self, label: str) -> bool:
        parts = self._split_label(label)
        if parts is None:
            return False
        _prefix, body = parts
        return bool(body) and body[-1] in ("B", "8", "6")

    def _label_has_confusable_character(self, label: str) -> bool:
        normalized = self._normalize_label(label)
        return any(char in normalized for char in ("B", "8", "6"))

    def _labels_differ_only_by_confusable_chars(self, left: str, right: str) -> bool:
        left_key = self._canonical_label(self._normalize_label(left))
        right_key = self._canonical_label(self._normalize_label(right))
        if not left_key or not right_key or len(left_key) != len(right_key):
            return False

        saw_difference = False
        for left_char, right_char in zip(left_key, right_key):
            if left_char == right_char:
                continue
            if not self._is_ambiguous_char_pair(left_char, right_char):
                return False
            saw_difference = True
        return saw_difference

    def _choose_char_with_ambiguity(
        self, votes: Dict[str, float]
    ) -> Tuple[str, float, float, bool]:
        if not votes:
            return "", 0.0, 0.0, False

        total = sum(votes.values())
        ordered = sorted(votes.items(), key=lambda item: item[1], reverse=True)
        best_char, best_weight = ordered[0]
        second_char = ordered[1][0] if len(ordered) > 1 else ""
        second_weight = ordered[1][1] if len(ordered) > 1 else 0.0
        ratio = best_weight / total if total > 0.0 else 0.0
        margin = (best_weight - second_weight) / total if total > 0.0 else 0.0
        ambiguous = (
            self._is_ambiguous_char_pair(best_char, second_char)
            and margin < self.ambiguous_char_margin
        )
        return best_char, ratio, margin, ambiguous

    def _vote_confirmed_candidate(self) -> Tuple[Optional[str], float, float, int, bool]:
        frames = [frame for frame in self.vote_window if frame]
        observations = len(frames)
        if observations < self.vote_min_observations:
            return None, 0.0, 0.0, observations, False

        parsed_frames = []
        max_prefix_len = 0
        max_body_len = 0
        for frame in frames:
            parsed_candidates = []
            for rank, (label, score, _box) in enumerate(frame[: self.max_labels_per_frame]):
                parts = self._split_label(label)
                if parts is None:
                    continue
                prefix, body = parts
                weight = self._candidate_weight(score, rank)
                parsed_candidates.append((prefix, body, weight))
                max_prefix_len = max(max_prefix_len, len(prefix))
                max_body_len = max(max_body_len, len(body))
            if parsed_candidates:
                parsed_frames.append(parsed_candidates)

        if not parsed_frames or max_prefix_len == 0 or max_body_len == 0:
            return None, 0.0, 0.0, observations, False

        prefix_chars = []
        char_ratios = []
        char_margins = []
        has_ambiguous_char = False
        for pos in range(max_prefix_len):
            votes: Dict[str, float] = {}
            for frame in parsed_frames:
                for prefix, _body, weight in frame:
                    offset = max_prefix_len - len(prefix)
                    if pos < offset:
                        continue
                    char = prefix[pos - offset]
                    votes[char] = votes.get(char, 0.0) + weight
            char, ratio, margin, ambiguous = self._choose_char_with_ambiguity(votes)
            if not char:
                return None, 0.0, 0.0, observations, has_ambiguous_char
            prefix_chars.append(char)
            char_ratios.append(ratio)
            char_margins.append(margin)
            has_ambiguous_char = has_ambiguous_char or ambiguous

        body_chars = []
        for pos in range(max_body_len):
            votes: Dict[str, float] = {}
            for frame in parsed_frames:
                for _prefix, body, weight in frame:
                    offset = max_body_len - len(body)
                    if pos < offset:
                        continue
                    char = body[pos - offset]
                    votes[char] = votes.get(char, 0.0) + weight
            char, ratio, margin, ambiguous = self._choose_char_with_ambiguity(votes)
            if not char:
                return None, 0.0, 0.0, observations, has_ambiguous_char
            body_chars.append(char)
            char_ratios.append(ratio)
            char_margins.append(margin)
            has_ambiguous_char = has_ambiguous_char or ambiguous

        candidate = self._format_label("".join(prefix_chars), "".join(body_chars))
        min_ratio = min(char_ratios) if char_ratios else 0.0
        min_margin = min(char_margins) if char_margins else 0.0
        return candidate, min_ratio, min_margin, observations, has_ambiguous_char

    def _update_confirmed_label(
        self, labels: List[Tuple[str, float, list]], guard_reason: str = ""
    ) -> Tuple[str, bool]:
        guarded = bool(guard_reason)
        appended_this_frame = False
        if labels and not guarded:
            self.vote_window.append(labels[: self.max_labels_per_frame])
            appended_this_frame = True

        candidate, ratio, margin, observations, ambiguous = self._vote_confirmed_candidate()
        effective_guard_reason = guard_reason
        if (
            not effective_guard_reason
            and self.guard_confusable_label_changes
            and candidate is not None
            and self.confirmed_label
            and not self._same_label(self.confirmed_label, candidate)
            and self._labels_differ_only_by_confusable_chars(
                self.confirmed_label, candidate
            )
        ):
            effective_guard_reason = "CONFUSABLE_CHANGE"
            if appended_this_frame and self.vote_window:
                self.vote_window.pop()
                candidate, ratio, margin, observations, ambiguous = (
                    self._vote_confirmed_candidate()
                )

        guarded = bool(effective_guard_reason)
        self.confirmation_guard_reason = effective_guard_reason
        self.needs_closer_view = ambiguous or guarded
        if candidate is not None and not ambiguous and not guarded:
            if not self.confirmed_label:
                if (
                    ratio >= self.vote_min_char_ratio
                    and margin >= self.vote_min_char_margin
                ):
                    self.confirmed_label = candidate
            elif self._same_label(self.confirmed_label, candidate):
                self.confirmed_label = candidate
            elif (
                observations >= self.confirmed_change_min_observations
                and ratio >= self.confirmed_change_min_char_ratio
                and margin >= self.confirmed_change_min_char_margin
            ):
                self.confirmed_label = candidate

        valid = bool(self.confirmed_label)
        self._publish_confirmed(self.confirmed_label, valid)
        self._publish_needs_closer_view(self.needs_closer_view)
        if valid and self.confirmed_label != self.last_logged_confirmed:
            self.get_logger().info(
                f"OCR confirmed: {self.confirmed_label} "
                f"(ratio={ratio:.2f}, margin={margin:.2f}, n={observations})"
            )
            self.last_logged_confirmed = self.confirmed_label
        if self.needs_closer_view != self.last_logged_needs_closer:
            self.get_logger().info(
                f"OCR needs_closer_view: {self.needs_closer_view} "
                f"(candidate={candidate or '<none>'}, ratio={ratio:.2f}, "
                f"margin={margin:.2f}, n={observations})"
            )
            self.last_logged_needs_closer = self.needs_closer_view
        if (
            effective_guard_reason
            and effective_guard_reason != self.last_logged_guard_reason
        ):
            self.get_logger().info(
                f"OCR confirmation_guard: {effective_guard_reason}"
            )
            self.last_logged_guard_reason = effective_guard_reason
        elif not effective_guard_reason and self.last_logged_guard_reason:
            self.get_logger().info("OCR confirmation_guard: clear")
            self.last_logged_guard_reason = ""
        return self.confirmed_label, valid

    def _extract_valid_labels(self, ocr_result) -> List[Tuple[str, float, list]]:
        best_by_label: Dict[str, Tuple[float, list, str]] = {}

        for box, text, score in self._iter_candidates(ocr_result):
            normalized = self._normalize_label(text)
            normalized = self._apply_format_hints(normalized)

            if score < self.conf_threshold:
                continue
            if not self.label_re.match(normalized):
                continue
            dedup_key = self._canonical_label(normalized)
            prev = best_by_label.get(dedup_key)
            if prev is None or score > prev[0]:
                best_by_label[dedup_key] = (score, box, normalized)

        labels = [(payload[2], payload[0], payload[1]) for payload in best_by_label.values()]
        labels.sort(key=lambda item: item[1], reverse=True)
        return labels[: self.max_labels_per_frame]

    def _pick_best_label(self, labels: List[Tuple[str, float, list]]) -> Tuple[Optional[str], float, Optional[list]]:
        if not labels:
            return None, 0.0, None
        label, score, box = labels[0]
        return label, score, box

    @staticmethod
    def _box_metrics(box: list) -> Optional[Tuple[float, float, float, float, float, float]]:
        if not box:
            return None

        points = []
        for point in box:
            if len(point) < 2:
                continue
            points.append((float(point[0]), float(point[1])))
        if not points:
            return None

        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        x0 = min(xs)
        y0 = min(ys)
        x1 = max(xs)
        y1 = max(ys)
        return x0, y0, x1, y1, (x0 + x1) * 0.5, (y0 + y1) * 0.5

    @staticmethod
    def _clamp_unit(value: float) -> float:
        return min(max(float(value), 0.0), 1.0)

    def _target_center(self) -> Tuple[float, float]:
        if self.target_roi is not None:
            x_min, y_min, x_max, y_max = self.target_roi
            return (x_min + x_max) * 0.5, (y_min + y_max) * 0.5
        return self._clamp_unit(self.target_center_x), self._clamp_unit(
            self.target_center_y
        )

    def _target_gate_half_size(self) -> Tuple[float, float]:
        if self.target_roi is not None:
            x_min, y_min, x_max, y_max = self.target_roi
            return (
                max(0.03, abs(x_max - x_min) * 0.5 + 0.03),
                max(0.03, abs(y_max - y_min) * 0.5 + 0.03),
            )
        return self.target_gate_half_width, self.target_gate_half_height

    def _target_priority_score(
        self, bgr: "cv2.Mat", label_item: Tuple[str, float, list]
    ) -> float:
        _label, score, box = label_item
        if not self.enable_target_door_priority:
            return float(score)

        metrics = self._box_metrics(box)
        if metrics is None:
            return float(score)

        x0, y0, x1, y1, cx, cy = metrics
        height, width = bgr.shape[:2]
        width = max(1, width)
        height = max(1, height)

        target_x, target_y = self._target_center()
        cx_norm = cx / width
        cy_norm = cy / height

        dx = abs(cx_norm - target_x) / 0.5
        dy = abs(cy_norm - target_y) / 0.5
        center_score = max(0.0, 1.0 - 0.85 * dx - 0.15 * dy)

        box_w = max(0.0, x1 - x0) / width
        box_h = max(0.0, y1 - y0) / height
        size_score = min(1.0, max(box_w, box_h) * 8.0)

        priority = (
            1.0
            + self.target_center_weight * center_score
            + self.target_size_weight * size_score
        )
        return float(score) * priority

    def _sort_labels_for_target(
        self, bgr: "cv2.Mat", labels: List[Tuple[str, float, list]]
    ) -> List[Tuple[str, float, list]]:
        if not labels:
            return []
        return sorted(
            labels,
            key=lambda item: (self._target_priority_score(bgr, item), item[1]),
            reverse=True,
        )

    def _primary_labels_for_target(
        self, bgr: "cv2.Mat", labels: List[Tuple[str, float, list]]
    ) -> List[Tuple[str, float, list]]:
        ordered = self._sort_labels_for_target(bgr, labels)
        if not self.enable_target_door_priority:
            return ordered
        return ordered[:1]

    def _confirmation_guard(
        self, bgr: "cv2.Mat", labels: List[Tuple[str, float, list]]
    ) -> str:
        if not self.enable_confirmation_guard or not labels:
            return ""

        ordered = self._sort_labels_for_target(bgr, labels)
        primary = ordered[0]
        primary_label, _primary_score, primary_box = primary
        metrics = self._box_metrics(primary_box)
        if metrics is not None:
            _x0, y0, _x1, y1, cx, cy = metrics
            image_h, image_w = bgr.shape[:2]
            cx_norm = cx / max(1, image_w)
            cy_norm = cy / max(1, image_h)
            target_x, target_y = self._target_center()
            gate_half_width, gate_half_height = self._target_gate_half_size()
            if (
                self.enable_target_gate
                and (
                    abs(cx_norm - target_x) > gate_half_width
                    or abs(cy_norm - target_y) > gate_half_height
                )
            ):
                return "OFF_TARGET"

            text_height = max(0.0, y1 - y0)
            min_height = self.confirm_min_ambiguous_text_height_px
            if not self._label_has_confusable_character(primary_label):
                min_height = self.confirm_min_text_height_px
            if text_height < min_height:
                return f"SMALL_TEXT:{text_height:.1f}px"

        if self.guard_on_multiple_signs:
            roi_bounds = self._crop_bounds_from_roi(bgr)
            if roi_bounds is not None:
                sign_bounds = self._sort_bounds_for_target(
                    bgr, self._detect_sign_bounds_in_roi(bgr, roi_bounds)
                )
                if len(sign_bounds) > 1:
                    primary_target_score = self._target_priority_score_for_bounds(
                        bgr, sign_bounds[0]
                    )
                    secondary_target_score = self._target_priority_score_for_bounds(
                        bgr, sign_bounds[1]
                    )
                    if (
                        primary_target_score > 0.0
                        and secondary_target_score
                        >= primary_target_score * self.target_ambiguity_ratio
                    ):
                        return "MULTI_SIGN"

        if len(ordered) > 1:
            primary_target_score = self._target_priority_score(bgr, ordered[0])
            for secondary in ordered[1:]:
                if self._same_label(primary_label, secondary[0]):
                    continue
                secondary_target_score = self._target_priority_score(bgr, secondary)
                if (
                    primary_target_score > 0.0
                    and secondary_target_score
                    >= primary_target_score * self.target_ambiguity_ratio
                ):
                    return "MULTI_TARGET"
                break

        return ""

    def _target_priority_score_for_bounds(
        self, bgr: "cv2.Mat", bounds: Tuple[int, int, int, int]
    ) -> float:
        if not self.enable_target_door_priority:
            return 0.0
        x0, y0, x1, y1 = bounds
        height, width = bgr.shape[:2]
        width = max(1, width)
        height = max(1, height)

        cx_norm = ((x0 + x1) * 0.5) / width
        cy_norm = ((y0 + y1) * 0.5) / height
        target_x, target_y = self._target_center()
        dx = abs(cx_norm - target_x) / 0.5
        dy = abs(cy_norm - target_y) / 0.5
        center_score = max(0.0, 1.0 - 0.85 * dx - 0.15 * dy)
        area_ratio = max(0.0, (x1 - x0) * (y1 - y0)) / float(width * height)
        compact_score = max(0.0, 1.0 - area_ratio * 35.0)
        readable_size_score = min(1.0, max((x1 - x0) / width, (y1 - y0) / height) * 18.0)
        return 1.8 * center_score + 0.55 * compact_score + 0.20 * readable_size_score

    def _sort_bounds_for_target(
        self, bgr: "cv2.Mat", bounds_list: List[Tuple[int, int, int, int]]
    ) -> List[Tuple[int, int, int, int]]:
        if not bounds_list:
            return []
        return sorted(
            bounds_list,
            key=lambda bounds: self._target_priority_score_for_bounds(bgr, bounds),
            reverse=True,
        )

    def _crop_bounds_from_box(self, bgr: "cv2.Mat", box: list) -> Optional[Tuple[int, int, int, int]]:
        if not box:
            return None

        height, width = bgr.shape[:2]
        points = []
        for point in box:
            if len(point) < 2:
                continue
            points.append((float(point[0]), float(point[1])))
        if not points:
            return None

        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        box_w = max(xs) - min(xs)
        box_h = max(ys) - min(ys)
        pad_x = max(self.crop_retry_padding_px, int(box_w * 0.75))
        pad_y = max(self.crop_retry_padding_px, int(box_h * 1.50))

        x0 = max(0, int(min(xs) - pad_x))
        y0 = max(0, int(min(ys) - pad_y))
        x1 = min(width, int(max(xs) + pad_x))
        y1 = min(height, int(max(ys) + pad_y))
        if x1 <= x0 or y1 <= y0:
            return None
        return x0, y0, x1, y1

    def _crop_bounds_from_roi(self, bgr: "cv2.Mat") -> Optional[Tuple[int, int, int, int]]:
        height, width = bgr.shape[:2]
        if self.target_roi is not None:
            x_min, y_min, x_max, y_max = self.target_roi
        else:
            x_min = min(max(self.roi_retry_x_min, 0.0), 1.0)
            x_max = min(max(self.roi_retry_x_max, 0.0), 1.0)
            y_min = min(max(self.roi_retry_y_min, 0.0), 1.0)
            y_max = min(max(self.roi_retry_y_max, 0.0), 1.0)

        x0 = int(min(x_min, x_max) * width)
        x1 = int(max(x_min, x_max) * width)
        y0 = int(min(y_min, y_max) * height)
        y1 = int(max(y_min, y_max) * height)
        if x1 <= x0 or y1 <= y0:
            return None
        return x0, y0, x1, y1

    def _candidate_bounds_from_ocr(
        self, bgr: "cv2.Mat", ocr_result
    ) -> List[Tuple[int, int, int, int]]:
        bounds_list: List[Tuple[int, int, int, int]] = []
        for box, _text, _score in self._iter_candidates(ocr_result):
            bounds = self._crop_bounds_from_box(bgr, box)
            if bounds is not None and bounds not in bounds_list:
                bounds_list.append(bounds)
        return bounds_list

    def _detect_sign_bounds_in_roi(
        self, bgr: "cv2.Mat", roi_bounds: Tuple[int, int, int, int]
    ) -> List[Tuple[int, int, int, int]]:
        if not self.enable_sign_detection_retry:
            return []

        x0, y0, x1, y1 = roi_bounds
        roi = bgr[y0:y1, x0:x1]
        if roi.size == 0:
            return []

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        masks = [
            # The sim labels are pale yellow plates; prefer them over white windows.
            (cv2.inRange(hsv, (10, 18, 130), (55, 220, 255)), 1.35),
            # Keep a stricter bright fallback for labels that render nearly white.
            (cv2.inRange(hsv, (0, 0, 165), (179, 75, 255)), 1.0),
        ]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        roi_area = max(1, roi.shape[0] * roi.shape[1])
        candidates = []
        seen_bounds = set()

        for mask, mask_bonus in masks:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                rx, ry, rw, rh = cv2.boundingRect(contour)
                if rw <= 0 or rh <= 0:
                    continue

                area = rw * rh
                area_ratio = area / roi_area
                if area < self.sign_retry_min_area_px:
                    continue
                if area_ratio > self.sign_retry_max_area_ratio:
                    continue

                aspect = rw / float(rh)
                if (
                    aspect < self.sign_retry_min_aspect
                    or aspect > self.sign_retry_max_aspect
                ):
                    continue

                width_ratio = rw / max(1, roi.shape[1])
                height_ratio = rh / max(1, roi.shape[0])
                if width_ratio > 0.34 or height_ratio > 0.13:
                    continue

                center_y = (ry + rh * 0.5) / max(1, roi.shape[0])
                if center_y < 0.18 or center_y > 0.92:
                    continue

                pad_x = max(5, int(rw * self.sign_retry_padding_ratio))
                pad_y = max(5, int(rh * self.sign_retry_padding_ratio))
                bx0 = max(0, x0 + rx - pad_x)
                by0 = max(0, y0 + ry - pad_y)
                bx1 = min(bgr.shape[1], x0 + rx + rw + pad_x)
                by1 = min(bgr.shape[0], y0 + ry + rh + pad_y)
                if bx1 <= bx0 or by1 <= by0:
                    continue

                bounds = (bx0, by0, bx1, by1)
                if bounds in seen_bounds:
                    continue
                seen_bounds.add(bounds)

                center_score = self._target_priority_score_for_bounds(bgr, bounds)
                compact_score = max(0.0, 1.0 - area_ratio * 45.0)
                score = mask_bonus * (2.0 * center_score + compact_score + aspect * 0.04)
                candidates.append((score, bounds))

        candidates.sort(key=lambda item: item[0], reverse=True)
        return [bounds for _score, bounds in candidates[: self.sign_retry_max_candidates]]

    def _tight_sign_plate_bounds(
        self, crop: "cv2.Mat"
    ) -> Optional[Tuple[int, int, int, int]]:
        if crop.size == 0:
            return None

        height, width = crop.shape[:2]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        masks = [
            cv2.inRange(hsv, (10, 18, 125), (55, 230, 255)),
            cv2.inRange(hsv, (0, 0, 170), (179, 85, 255)),
        ]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        crop_area = max(1, width * height)
        candidates = []

        for mask_idx, mask in enumerate(masks):
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                rx, ry, rw, rh = cv2.boundingRect(contour)
                if rw <= 0 or rh <= 0:
                    continue

                area = rw * rh
                area_ratio = area / crop_area
                if area < max(10, self.sign_retry_min_area_px // 2):
                    continue
                if area_ratio > 0.65:
                    continue

                aspect = rw / float(rh)
                if aspect < 2.0 or aspect > 14.0:
                    continue

                width_ratio = rw / max(1, width)
                height_ratio = rh / max(1, height)
                if width_ratio < 0.08 or height_ratio < 0.04:
                    continue
                if height_ratio > 0.45:
                    continue

                cx = (rx + rw * 0.5) / max(1, width)
                cy = (ry + rh * 0.5) / max(1, height)
                center_score = max(0.0, 1.0 - abs(cx - 0.5) - 0.35 * abs(cy - 0.5))
                compact_score = max(0.0, 1.0 - area_ratio * 2.2)
                color_bonus = 1.25 if mask_idx == 0 else 1.0
                score = color_bonus * (1.7 * center_score + compact_score + aspect * 0.03)
                candidates.append((score, (rx, ry, rw, rh)))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0], reverse=True)
        _score, (rx, ry, rw, rh) = candidates[0]
        pad_x = max(4, int(rw * 0.18))
        pad_y = max(4, int(rh * 0.35))
        x0 = max(0, rx - pad_x)
        y0 = max(0, ry - pad_y)
        x1 = min(width, rx + rw + pad_x)
        y1 = min(height, ry + rh + pad_y)
        if x1 <= x0 or y1 <= y0:
            return None
        return x0, y0, x1, y1

    @staticmethod
    def _parse_scale_candidates(raw_value, default_scale: float, max_scale: float) -> List[float]:
        if isinstance(raw_value, str):
            raw_items = [item.strip() for item in raw_value.split(",")]
        elif isinstance(raw_value, (list, tuple)):
            raw_items = raw_value
        else:
            raw_items = [raw_value]

        scales = []
        for raw_item in raw_items:
            if raw_item in ("", None):
                continue
            try:
                scale = float(raw_item)
            except (TypeError, ValueError):
                continue
            scale = min(max_scale, max(1.0, scale))
            if all(abs(scale - existing) > 0.01 for existing in scales):
                scales.append(scale)

        if not scales:
            scales.append(min(max_scale, max(1.0, float(default_scale))))
        return scales

    def _dynamic_crop_scale(self, crop: "cv2.Mat", default_scale: float) -> float:
        if self.crop_retry_scale_mode in ("fixed", "manual", "stepped", "step", "steps"):
            return max(1.0, float(default_scale))

        height = max(1, crop.shape[0])
        target_scale = self.crop_retry_target_height_px / float(height)
        scale = max(float(default_scale), target_scale)
        return min(self.crop_retry_max_scale, scale)

    def _crop_retry_scales(self, crop: "cv2.Mat", dynamic_scale: bool) -> List[float]:
        if self.crop_retry_scale_mode in ("stepped", "step", "steps"):
            return list(self.crop_retry_scale_candidates)

        scale = (
            self._dynamic_crop_scale(crop, self.crop_retry_scale)
            if dynamic_scale
            else self.crop_retry_scale
        )
        return [min(self.crop_retry_max_scale, max(1.0, float(scale)))]

    def _resize_crop_for_ocr(self, crop: "cv2.Mat", scale: float) -> "cv2.Mat":
        if scale <= 1.0:
            return crop
        return cv2.resize(
            crop,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_LANCZOS4,
        )

    @staticmethod
    def _enhance_gray_crop(crop: "cv2.Mat") -> "cv2.Mat":
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 5, 35, 35)
        clahe = cv2.createCLAHE(clipLimit=1.8, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)

        blurred = cv2.GaussianBlur(gray, (0, 0), sigmaX=0.8)
        return cv2.addWeighted(gray, 1.35, blurred, -0.35, 0)

    @staticmethod
    def _adaptive_block_size(gray: "cv2.Mat") -> int:
        min_dim = max(3, min(gray.shape[:2]))
        block = min(41, max(15, (min_dim // 2) * 2 + 1))
        if block >= min_dim:
            block = min_dim if min_dim % 2 == 1 else min_dim - 1
        return max(3, block)

    def _enhance_crop_for_ocr(self, crop: "cv2.Mat", scale: Optional[float] = None) -> "cv2.Mat":
        scale = self.crop_retry_scale if scale is None else max(1.0, float(scale))
        resized = self._resize_crop_for_ocr(crop, scale)
        gray = self._enhance_gray_crop(resized)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    def _crop_ocr_variants(
        self, crop: "cv2.Mat", scale: Optional[float] = None
    ) -> List[Tuple[str, "cv2.Mat"]]:
        scale = self.crop_retry_scale if scale is None else max(1.0, float(scale))
        resized = self._resize_crop_for_ocr(crop, scale)
        gray = self._enhance_gray_crop(resized)

        variants = [("sharp", cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))]
        if not self.enable_crop_retry_variants:
            return variants

        mode = self.crop_retry_mode
        if mode in ("sharp", "sharp_only"):
            return variants

        _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        if mode in ("otsu", "sharp_then_otsu", "all", "sharp_then_all"):
            variants.append(("otsu", cv2.cvtColor(otsu, cv2.COLOR_GRAY2BGR)))

        block = self._adaptive_block_size(gray)
        adaptive = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block,
            7,
        )
        if mode in ("adaptive", "sharp_then_adaptive", "all", "sharp_then_all"):
            variants.append(("adaptive", cv2.cvtColor(adaptive, cv2.COLOR_GRAY2BGR)))
        return variants

    def _make_crop_debug_image(
        self,
        crop: "cv2.Mat",
        scale: float,
        best_label: str = "",
        best_score: float = 0.0,
        best_variant: str = "",
    ) -> "cv2.Mat":
        height = max(1, crop.shape[0])
        debug_scale = max(scale, self.crop_debug_target_height_px / float(height))
        debug_scale = min(self.crop_retry_max_scale, debug_scale)
        resized = self._resize_crop_for_ocr(crop, debug_scale)

        lab = cv2.cvtColor(resized, cv2.COLOR_BGR2LAB)
        l_chan, a_chan, b_chan = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(8, 8))
        l_chan = clahe.apply(l_chan)
        display = cv2.cvtColor(cv2.merge((l_chan, a_chan, b_chan)), cv2.COLOR_LAB2BGR)
        display = cv2.bilateralFilter(display, 5, 25, 25)
        blurred = cv2.GaussianBlur(display, (0, 0), sigmaX=0.7)
        display = cv2.addWeighted(display, 1.25, blurred, -0.25, 0)

        title_h = 26
        canvas = cv2.copyMakeBorder(
            display,
            title_h,
            0,
            0,
            0,
            cv2.BORDER_CONSTANT,
            value=(18, 18, 18),
        )
        cv2.rectangle(canvas, (0, title_h), (canvas.shape[1] - 1, canvas.shape[0] - 1), (0, 255, 0), 2)

        if best_label:
            title = f"CROP {best_label} ({best_score:.2f}) {best_variant} x{scale:.1f}"
        else:
            title = f"CROP candidate x{scale:.1f}"
        cv2.putText(
            canvas,
            title[:80],
            (6, 19),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        return canvas

    def _map_crop_box_to_image(
        self, box: list, x0: int, y0: int, scale: float
    ) -> list:
        mapped = []
        for point in box:
            if len(point) < 2:
                continue
            mapped.append([x0 + float(point[0]) / scale, y0 + float(point[1]) / scale])
        return mapped

    def _publish_crop_debug(self, enhanced: "cv2.Mat", header) -> None:
        if not self.publish_debug_image:
            return
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(enhanced, encoding="bgr8")
            debug_msg.header = header
            self.crop_debug_pub.publish(debug_msg)
        except Exception as exc:
            self.get_logger().warn(f"crop debug publish failed: {exc}")

    def _run_crop_retry_on_bounds(
        self,
        bgr: "cv2.Mat",
        bounds: Tuple[int, int, int, int],
        header,
        dynamic_scale: bool = False,
        publish_debug: bool = True,
    ) -> List[Tuple[str, float, list]]:
        x0, y0, x1, y1 = bounds
        crop = bgr[y0:y1, x0:x1]
        if crop.size == 0:
            return []

        plate_bounds = self._tight_sign_plate_bounds(crop)
        if plate_bounds is not None:
            px0, py0, px1, py1 = plate_bounds
            crop = crop[py0:py1, px0:px1]
            x0 += px0
            y0 += py0
            x1 = x0 + crop.shape[1]
            y1 = y0 + crop.shape[0]
            if crop.size == 0:
                return []

        scale_candidates = self._crop_retry_scales(crop, dynamic_scale)
        best_debug_score = -1.0
        best_debug_label = ""
        best_debug_variant = ""
        best_debug_scale = scale_candidates[0]
        combined: Dict[str, Tuple[str, float, list, int]] = {}

        for scale in scale_candidates:
            variants = self._crop_ocr_variants(crop, scale=scale)
            best_scale_score = -1.0

            for variant_name, variant_img in variants:
                try:
                    crop_result = self._run_ocr(variant_img)
                except Exception as exc:
                    self.get_logger().warn(f"OCR crop retry failed: {exc}")
                    continue

                crop_labels = self._extract_valid_labels(crop_result)
                if crop_labels:
                    best_scale_score = max(best_scale_score, crop_labels[0][1])
                if crop_labels and crop_labels[0][1] > best_debug_score:
                    best_debug_score = crop_labels[0][1]
                    best_debug_label = crop_labels[0][0]
                    best_debug_variant = variant_name
                    best_debug_scale = scale

                for crop_label, crop_score, crop_box in crop_labels:
                    if crop_box is None or crop_score < self.crop_retry_accept_threshold:
                        continue
                    mapped_box = self._map_crop_box_to_image(crop_box, x0, y0, scale)
                    dedup_key = self._canonical_label(crop_label)
                    prev = combined.get(dedup_key)
                    if prev is None:
                        combined[dedup_key] = (
                            crop_label,
                            crop_score,
                            mapped_box or crop_box,
                            1,
                        )
                        continue

                    prev_label, prev_score, prev_box, prev_count = prev
                    if crop_score > prev_score:
                        prev_label = crop_label
                        prev_score = crop_score
                        prev_box = mapped_box or crop_box
                    combined[dedup_key] = (
                        prev_label,
                        prev_score,
                        prev_box,
                        prev_count + 1,
                    )

            if best_scale_score >= self.crop_retry_variant_min_score:
                break

        if publish_debug:
            debug_img = self._make_crop_debug_image(
                crop,
                best_debug_scale,
                best_debug_label,
                max(0.0, best_debug_score),
                best_debug_variant,
            )
            self._publish_crop_debug(debug_img, header)

        mapped_labels = []
        for crop_label, crop_score, crop_box, seen_count in combined.values():
            stable_score = min(1.0, crop_score + 0.03 * max(0, seen_count - 1))
            mapped_labels.append((crop_label, stable_score, crop_box))
        mapped_labels.sort(key=lambda item: item[1], reverse=True)
        return mapped_labels[: self.max_labels_per_frame]

    def _refine_labels_with_crop_retry(
        self, bgr: "cv2.Mat", labels: List[Tuple[str, float, list]], header
    ) -> List[Tuple[str, float, list]]:
        if not self.enable_crop_retry or not labels:
            return labels

        refined_by_label: Dict[str, Tuple[str, float, list]] = {}
        target_ordered_labels = self._sort_labels_for_target(bgr, labels)
        for index, (label, score, box) in enumerate(
            target_ordered_labels[: self.crop_retry_refine_max_candidates]
        ):
            dedup_key = self._canonical_label(label)
            refined_by_label[dedup_key] = (label, score, box)

            bounds = self._crop_bounds_from_box(bgr, box)
            if bounds is None:
                continue

            crop_labels = self._run_crop_retry_on_bounds(
                bgr,
                bounds,
                header,
                dynamic_scale=True,
                publish_debug=(index == 0),
            )
            for crop_label, crop_score, crop_box in crop_labels:
                crop_key = self._canonical_label(crop_label)
                prev = refined_by_label.get(crop_key)
                if prev is None or crop_score > prev[1]:
                    refined_by_label[crop_key] = (crop_label, crop_score, crop_box)

        refined_labels = list(refined_by_label.values())
        refined_labels = self._sort_labels_for_target(bgr, refined_labels)
        return refined_labels[: self.max_labels_per_frame]

    def _fallback_labels_with_roi_retry(
        self, bgr: "cv2.Mat", ocr_result, header
    ) -> List[Tuple[str, float, list]]:
        if not self.enable_crop_retry:
            return []

        bounds_list = []
        if self.enable_roi_retry:
            roi_bounds = self._crop_bounds_from_roi(bgr)
            if roi_bounds is not None:
                sign_bounds = self._detect_sign_bounds_in_roi(bgr, roi_bounds)
                for bounds in sign_bounds:
                    if bounds not in bounds_list:
                        bounds_list.append(bounds)

        if not bounds_list:
            bounds_list = self._candidate_bounds_from_ocr(bgr, ocr_result)
            if self.enable_roi_retry:
                roi_bounds = self._crop_bounds_from_roi(bgr)
                if roi_bounds is not None and roi_bounds not in bounds_list:
                    bounds_list.append(roi_bounds)

        bounds_list = self._sort_bounds_for_target(bgr, bounds_list)
        for index, bounds in enumerate(
            bounds_list[: self.crop_retry_fallback_max_candidates]
        ):
            labels = self._run_crop_retry_on_bounds(
                bgr,
                bounds,
                header,
                dynamic_scale=True,
                publish_debug=(index == 0),
            )
            if labels:
                return self._sort_labels_for_target(bgr, labels)
        return []

    def _iter_candidates(self, ocr_result):
        """Yield candidates in normalized tuple format: (box, text, score)."""
        if not ocr_result:
            return

        # RapidOCR style: [[box, text, score], ...]
        if isinstance(ocr_result, list) and ocr_result and isinstance(ocr_result[0], list):
            first = ocr_result[0]
            if len(first) >= 3 and isinstance(first[1], str):
                for row in ocr_result:
                    if len(row) < 3:
                        continue
                    yield row[0], str(row[1]), float(row[2])
                return

        # PaddleOCR style: [[ [box, [text, score]], ... ]]
        if isinstance(ocr_result, list) and ocr_result:
            rows = ocr_result[0] if isinstance(ocr_result[0], list) else []
            for line in rows:
                if len(line) < 2:
                    continue
                box = line[0]
                text = str(line[1][0])
                score = float(line[1][1])
                yield box, text, score

    def _publish_result(self, label: str, valid: bool, labels_all: List[str]) -> None:
        label_msg = String()
        label_msg.data = label
        self.label_pub.publish(label_msg)

        valid_msg = Bool()
        valid_msg.data = valid
        self.valid_pub.publish(valid_msg)

        all_msg = String()
        all_msg.data = ",".join(labels_all)
        self.labels_all_pub.publish(all_msg)

    def _reset_confirmation_state(self, reason: str = "manual") -> None:
        self.vote_window.clear()
        self.confirmed_label = ""
        self.needs_closer_view = False
        self.confirmation_guard_reason = ""
        self.last_logged_confirmed = ""
        self.last_logged_needs_closer = False
        self.last_logged_guard_reason = ""
        self._publish_confirmed("", False)
        self._publish_needs_closer_view(False)
        self.get_logger().info(f"OCR confirmation reset: {reason}")

    def _reset_confirmation_cb(self, _msg: Empty) -> None:
        self._reset_confirmation_state("topic")

    def _active_target_cb(self, msg: String) -> None:
        target_id = msg.data.strip()
        if target_id == self.active_target_id:
            return
        self.active_target_id = target_id
        self._reset_confirmation_state(
            f"active_target:{target_id if target_id else '<none>'}"
        )

    def _parse_target_roi(
        self, payload: str
    ) -> Optional[Tuple[float, float, float, float]]:
        text = payload.strip()
        if not text:
            return None

        parsed = json.loads(text)
        if isinstance(parsed, dict):
            x_min = parsed.get("x_min", parsed.get("xmin", parsed.get("x0")))
            y_min = parsed.get("y_min", parsed.get("ymin", parsed.get("y0")))
            x_max = parsed.get("x_max", parsed.get("xmax", parsed.get("x1")))
            y_max = parsed.get("y_max", parsed.get("ymax", parsed.get("y1")))
            values = [x_min, y_min, x_max, y_max]
        elif isinstance(parsed, list) and len(parsed) >= 4:
            values = parsed[:4]
        else:
            return None

        x_min, y_min, x_max, y_max = [self._clamp_unit(float(value)) for value in values]
        if abs(x_max - x_min) < 0.01 or abs(y_max - y_min) < 0.01:
            return None
        return x_min, y_min, x_max, y_max

    def _target_roi_cb(self, msg: String) -> None:
        payload = msg.data.strip()
        if payload.lower() in ("", "clear", "none", "null", "{}"):
            if self.target_roi is not None:
                self.target_roi = None
                self._reset_confirmation_state("target_roi:clear")
                self.get_logger().info("OCR target_roi cleared")
            return

        try:
            roi = self._parse_target_roi(payload)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Invalid OCR target_roi payload: {exc}")
            return

        if roi is None:
            self.get_logger().warn("Invalid OCR target_roi payload: expected x/y min/max")
            return
        if roi == self.target_roi:
            return

        self.target_roi = roi
        self._reset_confirmation_state("target_roi:update")
        self.get_logger().info(
            "OCR target_roi set: "
            f"x_min={roi[0]:.3f}, y_min={roi[1]:.3f}, "
            f"x_max={roi[2]:.3f}, y_max={roi[3]:.3f}"
        )

    def _publish_detections(
        self,
        bgr: "cv2.Mat",
        labels: List[Tuple[str, float, list]],
        primary_label: str,
        header=None,
    ) -> None:
        height, width = bgr.shape[:2]
        detections = []
        for index, (label, score, box) in enumerate(labels):
            metrics = self._box_metrics(box)
            if metrics is None:
                continue

            x0, y0, x1, y1, cx, cy = metrics
            center_px = [int(round(cx)), int(round(cy))]
            center_norm = [
                round(cx / max(1, width), 4),
                round(cy / max(1, height), 4),
            ]
            detections.append(
                {
                    "label": label,
                    "score": round(float(score), 3),
                    "bbox": [
                        int(round(x0)),
                        int(round(y0)),
                        int(round(x1)),
                        int(round(y1)),
                    ],
                    "bbox_center": center_px,
                    "center": center_norm,
                    "center_norm": center_norm,
                    "target_score": round(
                        self._target_priority_score(bgr, (label, score, box)), 3
                    ),
                    "rank": index,
                    "primary": index == 0 and label == primary_label,
                }
            )

        stamp = {"sec": 0, "nanosec": 0}
        frame_id = ""
        if header is not None:
            stamp = {
                "sec": int(getattr(header.stamp, "sec", 0)),
                "nanosec": int(getattr(header.stamp, "nanosec", 0)),
            }
            frame_id = str(getattr(header, "frame_id", ""))

        msg = String()
        msg.data = json.dumps(
            {
                "stamp": stamp,
                "frame_id": frame_id,
                "image_width": int(width),
                "image_height": int(height),
                "primary_label": primary_label,
                "confirmed_label": self.confirmed_label,
                "confirmed_valid": bool(self.confirmed_label),
                "needs_closer_view": bool(self.needs_closer_view),
                "confirmation_guard": self.confirmation_guard_reason,
                "active_target": self.active_target_id,
                "target_roi": list(self.target_roi) if self.target_roi else None,
                "detections": detections,
            },
            separators=(",", ":"),
        )
        self.detections_pub.publish(msg)

    def _publish_confirmed(self, label: str, valid: bool) -> None:
        label_msg = String()
        label_msg.data = label
        self.confirmed_label_pub.publish(label_msg)

        valid_msg = Bool()
        valid_msg.data = valid
        self.confirmed_valid_pub.publish(valid_msg)

    def _publish_needs_closer_view(self, needed: bool) -> None:
        msg = Bool()
        msg.data = needed
        self.needs_closer_view_pub.publish(msg)

    def _draw_debug(
        self,
        bgr: "cv2.Mat",
        labels: List[Tuple[str, float, list]],
        confirmed_label: str,
        confirmed_valid: bool,
        needs_closer_view: bool,
    ) -> "cv2.Mat":
        out = bgr.copy()
        for idx, (label, score, box) in enumerate(labels):
            pts = [(int(p[0]), int(p[1])) for p in box]
            color = (0, 255, 0) if idx == 0 else (255, 200, 0)
            thickness = 2 if idx == 0 else 1
            for i in range(len(pts)):
                cv2.line(out, pts[i], pts[(i + 1) % len(pts)], color, thickness)
            text_pos = (pts[0][0], max(20, pts[0][1] - 8))
            cv2.putText(
                out,
                f"{label} ({score:.2f})",
                text_pos,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )

        if confirmed_valid:
            cv2.putText(
                out,
                f"CONFIRMED: {confirmed_label}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.05,
                (0, 255, 0),
                3,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                out,
                "CONFIRMED: --",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.05,
                (0, 180, 255),
                3,
                cv2.LINE_AA,
            )

        if labels:
            label = labels[0][0]
            score = labels[0][1]
            labels_text = ",".join(item[0] for item in labels[:3])
            cv2.putText(
                out,
                f"RAW: {label} ({score:.2f}) [{labels_text}]",
                (20, 82),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                out,
                "RAW: NO_VALID_LABEL",
                (20, 82),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        if needs_closer_view:
            status_text = "STATUS: NEEDS_CLOSER_VIEW"
            if self.confirmation_guard_reason:
                status_text = f"{status_text} {self.confirmation_guard_reason}"
            cv2.putText(
                out,
                status_text[:80],
                (20, 118),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 180, 255),
                2,
                cv2.LINE_AA,
            )
        return out

    def _image_cb(self, msg: Image) -> None:
        self.seq += 1
        if self.seq % self.process_every_n != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        try:
            ocr_result = self._run_ocr(bgr)
        except Exception as exc:
            self.get_logger().warn(f"OCR backend failed: {exc}")
            return

        labels = self._extract_valid_labels(ocr_result)
        if labels:
            labels = self._refine_labels_with_crop_retry(bgr, labels, msg.header)
        else:
            labels = self._fallback_labels_with_roi_retry(bgr, ocr_result, msg.header)
        labels = self._sort_labels_for_target(bgr, labels)
        primary_labels = self._primary_labels_for_target(bgr, labels)
        guard_reason = self._confirmation_guard(bgr, labels)

        label, score, _ = self._pick_best_label(primary_labels)
        valid = label is not None
        labels_only = [item[0] for item in labels]
        confirmed_label, confirmed_valid = self._update_confirmed_label(
            primary_labels, guard_reason
        )
        self._publish_detections(bgr, labels, label or "", msg.header)

        if valid:
            if label != self.last_published_label:
                self.get_logger().info(f"OCR label: {label} ({score:.2f})")
                self.last_published_label = label
            all_joined = ",".join(labels_only)
            if all_joined != self.last_published_all:
                self.get_logger().info(f"OCR labels(all): {all_joined}")
                self.last_published_all = all_joined
            self._publish_result(label, True, labels_only)
        else:
            if self.last_published_all:
                self.get_logger().info("OCR labels(all): <none>")
                self.last_published_all = ""
            self._publish_result("", False, [])

        if self.publish_debug_image:
            debug_img = self._draw_debug(
                bgr, labels, confirmed_label, confirmed_valid, self.needs_closer_view
            )
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
            if self.legacy_debug_pub is not None:
                self.legacy_debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DoorOcrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
