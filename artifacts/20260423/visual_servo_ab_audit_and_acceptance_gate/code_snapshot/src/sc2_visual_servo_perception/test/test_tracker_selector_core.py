import unittest

from sc2_visual_servo_perception.tracker_selector_core import (
    DetectionCandidate,
    TRACK_CONFIRMED,
    TRACK_HOLD,
    associate_two_stage,
    compute_freshness,
    create_track,
    motion_gate_scale,
    select_track,
    update_track_with_match,
    update_track_with_miss,
)


def _candidate(u, v, score, high=True):
    return DetectionCandidate(
        detection=None,
        cls_id='3',
        score=score,
        u=float(u),
        v=float(v),
        w=30.0,
        h=30.0,
        high_confidence=high,
    )


class TrackerSelectorCoreTest(unittest.TestCase):
    def test_freshness_clamps(self):
        self.assertAlmostEqual(compute_freshness(10.0, 9.5, 1.0), 0.5)
        self.assertAlmostEqual(compute_freshness(10.0, 7.0, 1.0), 0.0)
        self.assertAlmostEqual(compute_freshness(10.0, 11.0, 1.0), 1.0)

    def test_motion_gate_scale_caps(self):
        scale = motion_gate_scale(
            1.0,
            1.0,
            v_ref_mps=0.5,
            omega_ref_radps=0.5,
            beta_v=0.3,
            beta_omega=0.3,
            r_gate_max_scale=1.6,
        )
        self.assertAlmostEqual(scale, 1.6)

    def test_two_stage_association_uses_low_confidence_for_unmatched_track(self):
        t1 = create_track(1, _candidate(10.0, 10.0, 0.9), 0.0)
        t2 = create_track(2, _candidate(200.0, 200.0, 0.9), 0.0)
        for tr in (t1, t2):
            tr.state = TRACK_CONFIRMED

        detections = [
            _candidate(12.0, 10.0, 0.92, high=True),
            _candidate(202.0, 201.0, 0.25, high=False),
        ]
        assignments, unmatched = associate_two_stage(
            [t1, t2],
            detections,
            gate_px=20.0,
            alpha_uv_predict=0.0,
        )

        self.assertEqual(assignments[1][1], 'high_confidence_match')
        self.assertEqual(assignments[2][1], 'low_confidence_recovery')
        self.assertEqual(unmatched, [])

    def test_confirm_hold_and_selection_over_track_table(self):
        t1 = create_track(1, _candidate(10.0, 10.0, 0.9), 0.0)
        t2 = create_track(2, _candidate(200.0, 200.0, 0.8), 0.0)
        update_track_with_match(
            t1,
            _candidate(11.0, 10.0, 0.9),
            0.1,
            n_confirm=2,
            confidence_ema_alpha=0.5,
            reason_code='high_confidence_match',
        )
        self.assertEqual(t1.state, TRACK_CONFIRMED)

        update_track_with_miss(
            t1,
            0.2,
            t_stale_sec=1.0,
            hold_max_misses=5,
            lost_max_misses=10,
        )
        self.assertEqual(t1.state, TRACK_HOLD)
        self.assertGreater(t1.freshness, 0.0)

        selected = select_track([t1, t2], 0.2, 1.0)
        self.assertEqual(selected.track_id, 1)

    def test_confirmed_track_enters_hold_when_stale_before_miss_limit(self):
        track = create_track(1, _candidate(10.0, 10.0, 0.9), 0.0)
        track.state = TRACK_CONFIRMED

        update_track_with_miss(
            track,
            2.0,
            t_stale_sec=1.0,
            hold_max_misses=5,
            lost_max_misses=10,
        )

        self.assertEqual(track.state, TRACK_HOLD)
        self.assertEqual(track.freshness, 0.0)

    def test_selection_continuity_keeps_previous_track_on_small_score_change(self):
        previous = create_track(1, _candidate(10.0, 10.0, 0.90), 0.0)
        challenger = create_track(2, _candidate(200.0, 200.0, 0.92), 0.0)
        previous.state = TRACK_CONFIRMED
        challenger.state = TRACK_CONFIRMED

        selected = select_track(
            [previous, challenger],
            0.1,
            1.0,
            previous_selected_track_id=1,
            selector_selected_track_bonus=0.03,
            selector_switch_margin=0.05,
        )

        self.assertEqual(selected.track_id, 1)

    def test_selection_continuity_switches_when_margin_is_exceeded(self):
        previous = create_track(1, _candidate(10.0, 10.0, 0.90), 0.0)
        challenger = create_track(2, _candidate(200.0, 200.0, 1.00), 0.0)
        previous.state = TRACK_CONFIRMED
        challenger.state = TRACK_CONFIRMED

        selected = select_track(
            [previous, challenger],
            0.1,
            1.0,
            previous_selected_track_id=1,
            selector_selected_track_bonus=0.03,
            selector_switch_margin=0.05,
        )

        self.assertEqual(selected.track_id, 2)


if __name__ == '__main__':
    unittest.main()
