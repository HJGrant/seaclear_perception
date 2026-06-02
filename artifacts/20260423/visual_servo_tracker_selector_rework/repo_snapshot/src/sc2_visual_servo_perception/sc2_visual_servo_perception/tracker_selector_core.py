import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


TRACK_TENTATIVE = 'TENTATIVE'
TRACK_CONFIRMED = 'CONFIRMED'
TRACK_HOLD = 'HOLD'
TRACK_LOST = 'LOST'
TRACK_ESCALATE = 'ESCALATE'


@dataclass
class DetectionCandidate:
    detection: object
    cls_id: str
    score: float
    u: float
    v: float
    w: float
    h: float
    high_confidence: bool


@dataclass
class TrackState:
    track_id: int
    state: str
    u: float
    v: float
    w: float
    h: float
    du: float = 0.0
    dv: float = 0.0
    age_frames: int = 0
    hit_count: int = 0
    hit_streak: int = 0
    miss_count: int = 0
    confidence_ema: float = 0.0
    freshness: float = 1.0
    last_match_time_sec: float = 0.0
    last_match_confidence: float = 0.0
    last_class_id: str = ''
    reason_code: str = 'new_track'


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def class_matches(candidate: str, target_class: str) -> bool:
    target = str(target_class).strip()
    if not target:
        return True
    cand = str(candidate).strip()
    if cand == target:
        return True
    try:
        return int(cand) == int(target)
    except (TypeError, ValueError):
        return False


def compute_freshness(now_sec: float, last_match_time_sec: float, t_stale_sec: float) -> float:
    if t_stale_sec <= 0.0:
        return 0.0
    return clamp01(1.0 - ((float(now_sec) - float(last_match_time_sec)) / float(t_stale_sec)))


def motion_gate_scale(
    linear_speed_mps: float,
    yaw_rate_radps: float,
    *,
    v_ref_mps: float,
    omega_ref_radps: float,
    beta_v: float,
    beta_omega: float,
    r_gate_max_scale: float,
) -> float:
    s_v = clamp01(float(linear_speed_mps) / max(1e-6, float(v_ref_mps)))
    s_omega = clamp01(abs(float(yaw_rate_radps)) / max(1e-6, float(omega_ref_radps)))
    scale = 1.0 + max(0.0, float(beta_v)) * s_v + max(0.0, float(beta_omega)) * s_omega
    return min(max(1.0, float(r_gate_max_scale)), scale)


def predicted_uv(track: TrackState, alpha_uv_predict: float) -> Tuple[float, float]:
    alpha = max(0.0, min(0.3, float(alpha_uv_predict)))
    return float(track.u + alpha * track.du), float(track.v + alpha * track.dv)


def detection_distance(track: TrackState, candidate: DetectionCandidate, alpha_uv_predict: float) -> float:
    pu, pv = predicted_uv(track, alpha_uv_predict)
    return math.hypot(float(candidate.u) - pu, float(candidate.v) - pv)


def create_track(track_id: int, candidate: DetectionCandidate, now_sec: float) -> TrackState:
    return TrackState(
        track_id=int(track_id),
        state=TRACK_TENTATIVE,
        u=float(candidate.u),
        v=float(candidate.v),
        w=float(candidate.w),
        h=float(candidate.h),
        age_frames=1,
        hit_count=1,
        hit_streak=1,
        confidence_ema=float(candidate.score),
        last_match_time_sec=float(now_sec),
        last_match_confidence=float(candidate.score),
        last_class_id=str(candidate.cls_id),
        reason_code='new_track',
    )


def update_track_with_match(
    track: TrackState,
    candidate: DetectionCandidate,
    now_sec: float,
    *,
    n_confirm: int,
    confidence_ema_alpha: float,
    reason_code: str,
) -> None:
    prev_u, prev_v = track.u, track.v
    track.u = float(candidate.u)
    track.v = float(candidate.v)
    track.w = float(candidate.w)
    track.h = float(candidate.h)
    track.du = track.u - prev_u
    track.dv = track.v - prev_v
    track.age_frames += 1
    track.hit_count += 1
    track.hit_streak += 1
    track.miss_count = 0
    alpha = clamp01(confidence_ema_alpha)
    track.confidence_ema = alpha * float(candidate.score) + (1.0 - alpha) * track.confidence_ema
    track.last_match_time_sec = float(now_sec)
    track.last_match_confidence = float(candidate.score)
    track.last_class_id = str(candidate.cls_id)
    track.reason_code = str(reason_code)
    track.freshness = 1.0
    if track.state in {TRACK_CONFIRMED, TRACK_HOLD}:
        track.state = TRACK_CONFIRMED
    elif track.hit_streak >= max(1, int(n_confirm)):
        track.state = TRACK_CONFIRMED
    else:
        track.state = TRACK_TENTATIVE


def update_track_with_miss(
    track: TrackState,
    now_sec: float,
    *,
    t_stale_sec: float,
    hold_max_misses: int,
    lost_max_misses: int,
) -> None:
    track.age_frames += 1
    track.miss_count += 1
    track.hit_streak = 0
    freshness = compute_freshness(now_sec, track.last_match_time_sec, t_stale_sec)
    track.freshness = freshness
    if track.miss_count >= max(1, int(lost_max_misses)):
        track.state = TRACK_ESCALATE
        track.reason_code = 'escalate_miss_limit'
    elif track.miss_count > max(0, int(hold_max_misses)):
        track.state = TRACK_LOST
        track.reason_code = 'lost_stale'
    elif track.state in {TRACK_CONFIRMED, TRACK_HOLD}:
        track.state = TRACK_HOLD
        track.reason_code = 'hold_no_match'
    else:
        track.state = TRACK_TENTATIVE
        track.reason_code = 'tentative_no_match'


def assign_greedy(
    tracks: Sequence[TrackState],
    candidates: Sequence[DetectionCandidate],
    *,
    gate_px: float,
    alpha_uv_predict: float,
) -> Tuple[Dict[int, int], List[int], List[int]]:
    costs: List[Tuple[float, int, int]] = []
    for track in tracks:
        if track.state in {TRACK_LOST, TRACK_ESCALATE}:
            continue
        for cand_idx, candidate in enumerate(candidates):
            dist = detection_distance(track, candidate, alpha_uv_predict)
            if dist <= float(gate_px):
                costs.append((dist, int(track.track_id), cand_idx))

    assignments: Dict[int, int] = {}
    used_candidates = set()
    for _dist, track_id, cand_idx in sorted(costs, key=lambda x: x[0]):
        if track_id in assignments or cand_idx in used_candidates:
            continue
        assignments[track_id] = cand_idx
        used_candidates.add(cand_idx)

    unmatched_tracks = [
        int(t.track_id)
        for t in tracks
        if t.state not in {TRACK_LOST, TRACK_ESCALATE} and int(t.track_id) not in assignments
    ]
    unmatched_candidates = [idx for idx in range(len(candidates)) if idx not in used_candidates]
    return assignments, unmatched_tracks, unmatched_candidates


def associate_two_stage(
    tracks: Sequence[TrackState],
    candidates: Sequence[DetectionCandidate],
    *,
    gate_px: float,
    alpha_uv_predict: float,
) -> Tuple[Dict[int, Tuple[DetectionCandidate, str]], List[int]]:
    high_pairs = [(idx, c) for idx, c in enumerate(candidates) if c.high_confidence]
    low_pairs = [(idx, c) for idx, c in enumerate(candidates) if not c.high_confidence]
    high_candidates = [c for _idx, c in high_pairs]
    low_candidates = [c for _idx, c in low_pairs]

    high_assignments, unmatched_track_ids, _high_unmatched = assign_greedy(
        tracks,
        high_candidates,
        gate_px=gate_px,
        alpha_uv_predict=alpha_uv_predict,
    )

    result: Dict[int, Tuple[DetectionCandidate, str]] = {
        track_id: (high_candidates[cand_idx], 'high_confidence_match')
        for track_id, cand_idx in high_assignments.items()
    }

    track_by_id = {int(t.track_id): t for t in tracks}
    low_tracks = [track_by_id[tid] for tid in unmatched_track_ids if tid in track_by_id]
    low_assignments, _low_unmatched_track_ids, _low_unmatched = assign_greedy(
        low_tracks,
        low_candidates,
        gate_px=gate_px,
        alpha_uv_predict=alpha_uv_predict,
    )
    for track_id, cand_idx in low_assignments.items():
        result[track_id] = (low_candidates[cand_idx], 'low_confidence_recovery')

    unmatched_candidate_ids: List[int] = []
    high_used = set(high_assignments.values())
    for high_idx, (original_idx, _candidate) in enumerate(high_pairs):
        if high_idx not in high_used:
            unmatched_candidate_ids.append(original_idx)
    low_used = set(low_assignments.values())
    for low_idx, (original_idx, _candidate) in enumerate(low_pairs):
        if low_idx not in low_used:
            unmatched_candidate_ids.append(original_idx)

    return result, unmatched_candidate_ids


def selection_score(track: TrackState, now_sec: float, t_stale_sec: float) -> float:
    freshness = compute_freshness(now_sec, track.last_match_time_sec, t_stale_sec)
    if track.state == TRACK_CONFIRMED and freshness > 0.0:
        state_score = 3.0
    elif track.state == TRACK_HOLD and freshness > 0.0:
        state_score = 2.0
    elif track.state == TRACK_TENTATIVE:
        state_score = 1.0
    else:
        state_score = 0.0
    hit_score = min(max(0, int(track.hit_count)), 100) * 0.001
    return 10.0 * state_score + freshness + float(track.confidence_ema) + hit_score


def _is_confirmed_fresh(track: TrackState, now_sec: float, t_stale_sec: float) -> bool:
    return (
        track.state == TRACK_CONFIRMED and
        compute_freshness(now_sec, track.last_match_time_sec, t_stale_sec) > 0.0
    )


def select_track(
    tracks: Iterable[TrackState],
    now_sec: float,
    t_stale_sec: float,
    *,
    previous_selected_track_id: Optional[int] = None,
    selector_switch_margin: float = 0.0,
    selector_selected_track_bonus: float = 0.0,
) -> Optional[TrackState]:
    candidates = [t for t in tracks if t.state not in {TRACK_LOST, TRACK_ESCALATE}]
    if not candidates:
        return None

    base_scores = {
        int(track.track_id): selection_score(track, now_sec, t_stale_sec)
        for track in candidates
    }
    best = max(candidates, key=lambda track: (base_scores[int(track.track_id)], int(track.hit_count)))

    previous = None
    if previous_selected_track_id is not None:
        for track in candidates:
            if int(track.track_id) == int(previous_selected_track_id):
                previous = track
                break

    if previous is None or not _is_confirmed_fresh(previous, now_sec, t_stale_sec):
        return best

    previous_score = (
        base_scores[int(previous.track_id)] +
        max(0.0, float(selector_selected_track_bonus))
    )
    switch_threshold = previous_score + max(0.0, float(selector_switch_margin))
    best_other = None
    for track in candidates:
        if int(track.track_id) == int(previous.track_id):
            continue
        if best_other is None:
            best_other = track
            continue
        if (
            base_scores[int(track.track_id)],
            int(track.hit_count),
        ) > (
            base_scores[int(best_other.track_id)],
            int(best_other.hit_count),
        ):
            best_other = track

    if best_other is not None and base_scores[int(best_other.track_id)] > switch_threshold:
        return best_other
    return previous
