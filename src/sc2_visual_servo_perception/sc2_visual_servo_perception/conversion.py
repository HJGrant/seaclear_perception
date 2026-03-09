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


def compute_representative_depth_near_uv_median(
    depth_img: np.ndarray,
    u: float,
    v: float,
    w: float,
    h: float,
    *,
    min_depth_m: float = 0.2,
    max_depth_m: float = 20.0,
    min_valid_points: int = 20,
    num_nearest_points: int = 25,
    max_search_radius_px: float = 40.0,
    use_uv_median_depth: bool = True,
    mask_img: Optional[np.ndarray] = None,
    mask_threshold: int = 1,
) -> Tuple[float, float, float, int]:
    """Compute representative (u,v,z) from bbox ROI using local median depth near UV median.

    Returns:
      (u_rep, v_rep, z_rep_m, valid_points_in_roi). NaNs if insufficient valid points.
    """
    if depth_img is None:
        return float('nan'), float('nan'), float('nan'), 0

    img_h, img_w = depth_img.shape[:2]
    x0 = int(max(0, min(img_w - 1, round(u - 0.5 * w))))
    y0 = int(max(0, min(img_h - 1, round(v - 0.5 * h))))
    x1 = int(max(0, min(img_w, round(u + 0.5 * w))))
    y1 = int(max(0, min(img_h, round(v + 0.5 * h))))
    if x1 <= x0 or y1 <= y0:
        return float('nan'), float('nan'), float('nan'), 0

    roi = depth_img[y0:y1, x0:x1]
    if roi.size == 0:
        return float('nan'), float('nan'), float('nan'), 0

    if np.issubdtype(roi.dtype, np.integer):
        depth_m = roi.astype(np.float32) * 1e-3
    else:
        depth_m = roi.astype(np.float32)

    valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m >= float(min_depth_m)) & (
        depth_m <= float(max_depth_m)
    )

    if mask_img is not None:
        if mask_img.shape[:2] == depth_img.shape[:2]:
            mask_roi = mask_img[y0:y1, x0:x1]
            valid &= mask_roi >= int(mask_threshold)

    if not np.any(valid):
        return float('nan'), float('nan'), float('nan'), 0

    yy, xx = np.indices((y1 - y0, x1 - x0))
    u_all = xx.astype(np.float32) + float(x0)
    v_all = yy.astype(np.float32) + float(y0)

    u_valid = u_all[valid]
    v_valid = v_all[valid]
    z_valid = depth_m[valid]
    valid_count = int(z_valid.size)

    if valid_count < int(min_valid_points):
        return float('nan'), float('nan'), float('nan'), valid_count

    u_med = float(np.median(u_valid))
    v_med = float(np.median(v_valid))

    if not use_uv_median_depth:
        return u_med, v_med, float(np.median(z_valid)), valid_count

    du = u_valid - u_med
    dv = v_valid - v_med
    dist2 = du * du + dv * dv

    if max_search_radius_px > 0.0:
        within = dist2 <= float(max_search_radius_px) ** 2
    else:
        within = np.ones_like(dist2, dtype=bool)

    if np.any(within):
        candidate_idx = np.where(within)[0]
    else:
        candidate_idx = np.arange(valid_count, dtype=np.int64)

    if candidate_idx.size == 0:
        return float('nan'), float('nan'), float('nan'), valid_count

    k = max(1, int(num_nearest_points))
    if candidate_idx.size > k:
        d = dist2[candidate_idx]
        k_small = np.argpartition(d, k - 1)[:k]
        sel_idx = candidate_idx[k_small]
    else:
        sel_idx = candidate_idx

    u_rep = float(np.median(u_valid[sel_idx]))
    v_rep = float(np.median(v_valid[sel_idx]))
    z_rep = float(np.median(z_valid[sel_idx]))
    return u_rep, v_rep, z_rep, valid_count
