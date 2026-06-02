# Known Limitations

- 初期版は精密 ego-motion compensation を実装していない。
- motion-aware gate expansion は近接域で精度低下しうる。
- CameraInfo は水中精密幾何の truth ではない。
- target_point path は今回は主経路に入れていない。
- association は greedy であり Hungarian ではない。
- appearance feature や class-specific target priority は未実装。
- selector continuity は bonus + switch margin 方式であり、appearance matching ではない。
- 明示的 `REACQUIRE` state は未導入であり、現実装では `HOLD` が passive reacquire を兼ねる。
- `LOST` / `ESCALATE` 後の再取得は new track として扱うため、track_id continuity は失われる。
- HOLD maintain は depth/distance maintain と body velocity damping であり、full pose hold ではない。
- localization setpoint による horizontal station keeping は未実装。
- DVL/altitude が stale の場合、HOLD maintain は vertical zero に戻る。
- body velocity measurement が stale の場合、HOLD maintain は XY/yaw zero に戻る。
- `/detections_output` の header frame が bag では空だったため、tracker は pixel coordinate contract として扱っている。
- threshold/gate/hold parameters は仮初期値であり、field bag replay と実機で調整が必要。
- ROS integration test と colcon build はこの host では実行できていない。
