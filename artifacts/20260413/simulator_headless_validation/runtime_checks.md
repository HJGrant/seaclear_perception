# Runtime Checks

## Status
- Not executed in this environment (Docker/ROS missing).

## Expected outputs (once run inside container)
- Phase 1 simulator: `/sc2/visual_servo/target_point` publishes finite values.
- Phase 1 simulator: `/sc2/visual_servo/cmd_vel_body` publishes.
- Phase 1 simulator: `/sim/odom` changes over time.
- IBVS reference: `/mavros/setpoint_velocity/cmd_vel` publishes.
