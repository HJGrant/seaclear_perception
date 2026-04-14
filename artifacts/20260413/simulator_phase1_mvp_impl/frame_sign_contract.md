# Frame / Sign Contract

## Body frame
- `base_link`
- right-handed
- `+x`: forward
- `+y`: left
- `+z`: up

## Camera frames
- `down_camera_link`
  - helper frame colocated with camera mount
- `down_camera_optical_frame`
  - right-handed optical frame for downward-looking camera
  - mapping from body:
    - `x_opt = -y_body`
    - `y_opt = -x_body`
    - `z_opt = -z_body`

## Image axes
- image `u`: right positive
- image `v`: down positive

## Command sign table

| Case | Image / depth observation | Expected body correction | Expected command sign |
| --- | --- | --- | --- |
| Left/right test | target appears right of center, `u_error > 0` | move vehicle right | `linear.y < 0` |
| Left/right test | target appears left of center, `u_error < 0` | move vehicle left | `linear.y > 0` |
| Front/back test | target appears below center, `v_error > 0` | move vehicle forward | `linear.x > 0` |
| Front/back test | target appears above center, `v_error < 0` | move vehicle backward | `linear.x < 0` |
| Height test | target depth larger than desired | move vehicle down | `linear.z < 0` |
| Height test | target depth smaller than desired | move vehicle up | `linear.z > 0` |

## Static implementation references
- `sim_static_frames_node`: body to optical transform is defined in code.
- `sim_downward_camera_node`: projection uses the optical mapping above.
- `visual_servo_controller_node`: staged PID maps image error into `cmd_vel_body`.

## Runtime sign tests to execute
1. Start with positive lateral offset and confirm `/sc2/visual_servo/cmd_vel_body.twist.linear.y < 0`.
2. Start with positive forward image error and confirm `.linear.x > 0`.
3. Start above desired altitude and confirm `.linear.z < 0`.
