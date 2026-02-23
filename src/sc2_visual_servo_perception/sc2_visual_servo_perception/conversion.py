import math
from typing import Optional, Tuple

import numpy as np


def extract_hypothesis(result) -> Tuple[str, float]:
    """Extract (class_id, score) from a Detection2D result hypothesis."""
    cls_id = ''
    score = 0.0

    hypothesis = getattr(result, 'hypothesis', None)
    if hypothesis is not None:
        cls_raw = getattr(hypothesis, 'class_id', '')
        cls_id = str(cls_raw)
        score = float(getattr(hypothesis, 'score', 0.0))
    else:
        cls_raw = getattr(result, 'id', '')
        cls_id = str(cls_raw)
        score = float(getattr(result, 'score', 0.0))

    return cls_id, score


def _class_matches(candidate: str, target_class: str) -> bool:
    target = str(target_class).strip()
    if not target:
        return True

    cand = str(candidate).strip()
    if cand == target:
        return True

    # Allow numeric string/int equivalence when possible.
    try:
        return int(cand) == int(target)
    except (TypeError, ValueError):
        return False


def select_best_detection(detections_msg, target_class: str, min_confidence: float, pick_strategy: str):
    """Return best Detection2D according to strategy, class and confidence."""
    best_det = None
    best_metric = -math.inf
    strategy = str(pick_strategy).strip().lower()
    if strategy not in {'highest_confidence', 'largest_bbox'}:
        strategy = 'highest_confidence'

    for det in detections_msg.detections:
        if not det.results:
            continue
        cls_id, score = extract_hypothesis(det.results[0])
        if score < float(min_confidence):
            continue
        if not _class_matches(cls_id, target_class):
            continue

        area = float(det.bbox.size_x) * float(det.bbox.size_y)
        metric = score if strategy == 'highest_confidence' else area
        if metric > best_metric:
            best_det = det
            best_metric = metric

    return best_det


def compute_depth_median(depth_img: np.ndarray, u: float, v: float, w: float, h: float) -> float:
    """Compute robust median depth inside bbox ROI (meters), NaN if invalid."""
    if depth_img is None:
        return float('nan')

    img_h, img_w = depth_img.shape[:2]
    x0 = int(max(0, min(img_w - 1, round(u - 0.5 * w))))
    y0 = int(max(0, min(img_h - 1, round(v - 0.5 * h))))
    x1 = int(max(0, min(img_w, round(u + 0.5 * w))))
    y1 = int(max(0, min(img_h, round(v + 0.5 * h))))
    if x1 <= x0 or y1 <= y0:
        return float('nan')

    roi = depth_img[y0:y1, x0:x1]
    if roi.size == 0:
        return float('nan')

    if np.issubdtype(roi.dtype, np.integer):
        depth_m = roi.astype(np.float32) * 1e-3
    else:
        depth_m = roi.astype(np.float32)

    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    if not np.any(valid):
        return float('nan')

    return float(np.median(depth_m[valid]))
