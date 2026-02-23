import math
from types import SimpleNamespace

import numpy as np

from sc2_visual_servo_perception.conversion import compute_depth_median, select_best_detection


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
