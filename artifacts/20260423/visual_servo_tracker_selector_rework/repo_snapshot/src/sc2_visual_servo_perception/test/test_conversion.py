import math
from types import SimpleNamespace

import numpy as np

from sc2_visual_servo_perception.conversion import (
    compute_depth_median,
    compute_representative_depth_near_uv_median,
    select_best_detection,
)


def _make_det(cls_id: str, score: float, u: float = 320.0, v: float = 240.0):
    hyp = SimpleNamespace(hypothesis=SimpleNamespace(class_id=cls_id, score=score))
    bbox = SimpleNamespace(
        center=SimpleNamespace(position=SimpleNamespace(x=u, y=v)),
        size_x=100.0,
        size_y=80.0,
    )
    return SimpleNamespace(results=[hyp], bbox=bbox)


def test_select_best_detection_filters_class_and_score():
    msg = SimpleNamespace(detections=[
        _make_det('bottle', 0.4),
        _make_det('can', 0.9),
        _make_det('bottle', 0.8),
    ])

    best = select_best_detection(
        msg, target_class='bottle', min_confidence=0.5, pick_strategy='highest_confidence'
    )
    assert best is not None
    assert abs(best.bbox.center.position.x - 320.0) < 1e-6


def test_compute_depth_median_uint16_mm():
    depth = np.zeros((8, 8), dtype=np.uint16)
    depth[2:6, 2:6] = 1500
    val = compute_depth_median(depth, u=4, v=4, w=4, h=4)
    assert abs(val - 1.5) < 1e-6


def test_compute_depth_median_no_valid_returns_nan():
    depth = np.zeros((4, 4), dtype=np.float32)
    val = compute_depth_median(depth, u=2, v=2, w=2, h=2)
    assert math.isnan(val)


def test_representative_depth_uses_uv_median_neighborhood():
    depth = np.full((11, 11), 5000, dtype=np.uint16)
    depth[4:7, 4:7] = 1200

    u_rep, v_rep, z_rep, count = compute_representative_depth_near_uv_median(
        depth,
        u=5.0,
        v=5.0,
        w=11.0,
        h=11.0,
        min_depth_m=0.2,
        max_depth_m=20.0,
        min_valid_points=10,
        num_nearest_points=9,
        max_search_radius_px=3.0,
        use_uv_median_depth=True,
    )
    assert count >= 10
    assert abs(u_rep - 5.0) <= 1.0
    assert abs(v_rep - 5.0) <= 1.0
    assert abs(z_rep - 1.2) < 1e-3


def test_representative_depth_respects_min_valid_points():
    depth = np.zeros((7, 7), dtype=np.uint16)
    depth[3, 3] = 1000

    _, _, z_rep, count = compute_representative_depth_near_uv_median(
        depth,
        u=3.0,
        v=3.0,
        w=7.0,
        h=7.0,
        min_depth_m=0.2,
        max_depth_m=20.0,
        min_valid_points=5,
        num_nearest_points=3,
        max_search_radius_px=2.0,
        use_uv_median_depth=True,
    )
    assert count == 1
    assert math.isnan(z_rep)
