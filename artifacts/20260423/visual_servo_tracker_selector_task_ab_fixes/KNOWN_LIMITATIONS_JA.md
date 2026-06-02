# Known Limitations

- `ros2` / `colcon` がこの host に無いため、clean ROS build は未確認。
- selector continuity は score-based hysteresis であり、appearance matching ではない。
- `selector_switch_margin` と `selector_selected_track_bonus` は仮初期値であり、bag replay / 実機で調整が必要。
- 明示的な `REACQUIRE` state は未導入。現実装では `HOLD` が passive reacquire を兼ねる。
- `LOST` / `ESCALATE` 後の再取得は new track であり、track id continuity は失われる。
- Hungarian / global assignment は未実装。
- full 6-DoF hold、localization setpoint station keeping、precise underwater reprojection、P5 target_point 再接続は今回スコープ外。
- `package.xml` 依存は source import に基づき修正したが、実際の apt/rosdep key availability は Task C の clean environment で確認が必要。
