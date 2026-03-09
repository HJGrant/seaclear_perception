import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sc2_visual_servo_interfaces.msg import TargetObservation
from std_msgs.msg import String

from .ibvs.controller_math import (
    BBoxApproachConfig,
    IBVSConfig,
    Observation,
    clamp,
    compute_bbox_approach_command,
    compute_ibvs_command,
)

try:
    from .pbvs.mppi_core import MPPIController, axis_to_index, build_linear_map_from_axes
    _MPPI_AVAILABLE = True
except Exception:
    _MPPI_AVAILABLE = False


class VisualServoControllerNode(Node):
    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def __init__(self):
        super().__init__('visual_servo_controller_node')

        self.declare_parameter('target_topic', '/sc2/visual_servo/target_obs')
        self.declare_parameter('grasp_pose_topic', '/sc2/visual_servo/grasp_pose')
        self.declare_parameter('cmd_topic', '/sc2/visual_servo/cmd_vel_body')
        self.declare_parameter('status_topic', '/sc2/visual_servo/status')
        self.declare_parameter('body_frame_id', 'base_link')
        self.declare_parameter('timeout_ms', 500)
        self.declare_parameter('grasp_timeout_ms', 500)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('controller_mode', 'ibvs')

        self.declare_parameter('image_width', 1280.0)
        self.declare_parameter('image_height', 720.0)
        self.declare_parameter('target_u', 640.0)
        self.declare_parameter('target_v', 360.0)
        self.declare_parameter('desired_depth_m', 1.0)
        self.declare_parameter('desired_bbox_area_px', 18000.0)
        self.declare_parameter('k_u', 0.6)
        self.declare_parameter('k_v', 0.6)
        self.declare_parameter('k_depth', 0.5)
        self.declare_parameter('k_area', 0.3)
        self.declare_parameter('k_yaw', 0.8)
        self.declare_parameter('bbox_use_vy', True)
        self.declare_parameter('bbox_use_vz', True)
        self.declare_parameter('bbox_stop_area_px', 18000.0)
        self.declare_parameter('k_grasp_distance', 0.6)
        self.declare_parameter('desired_grasp_distance_m', 0.6)
        self.declare_parameter('grasp_distance_mode', 'z')
        self.declare_parameter('require_grasp_pose', False)

        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_vz', 0.3)
        self.declare_parameter('max_yaw_rate', 0.4)
        self.declare_parameter('slew_vx', 2.0)
        self.declare_parameter('slew_vy', 2.0)
        self.declare_parameter('slew_vz', 2.0)
        self.declare_parameter('slew_yaw_rate', 2.0)

        self.declare_parameter('fx_px', 600.0)
        self.declare_parameter('fy_px', 600.0)
        self.declare_parameter('cx_px', 640.0)
        self.declare_parameter('cy_px', 360.0)

        self._target_topic = str(self.get_parameter('target_topic').value)
        self._grasp_pose_topic = str(self.get_parameter('grasp_pose_topic').value)
        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._body_frame_id = str(self.get_parameter('body_frame_id').value)
        self._timeout_sec = float(self.get_parameter('timeout_ms').value) * 1e-3
        self._grasp_timeout_sec = float(self.get_parameter('grasp_timeout_ms').value) * 1e-3
        self._controller_mode = str(self.get_parameter('controller_mode').value).strip().lower()
        if self._controller_mode not in {'ibvs', 'bbox_approach', 'bbox_grasp', 'mppi'}:
            self.get_logger().warn(f'Unknown controller_mode={self._controller_mode}; falling back to ibvs')
            self._controller_mode = 'ibvs'

        self._cfg = IBVSConfig(
            image_width=float(self.get_parameter('image_width').value),
            image_height=float(self.get_parameter('image_height').value),
            target_u=float(self.get_parameter('target_u').value),
            target_v=float(self.get_parameter('target_v').value),
            desired_depth_m=float(self.get_parameter('desired_depth_m').value),
            desired_bbox_area_px=float(self.get_parameter('desired_bbox_area_px').value),
            k_u=float(self.get_parameter('k_u').value),
            k_v=float(self.get_parameter('k_v').value),
            k_depth=float(self.get_parameter('k_depth').value),
            k_area=float(self.get_parameter('k_area').value),
            k_yaw=float(self.get_parameter('k_yaw').value),
        )

        self._max_vx = float(self.get_parameter('max_vx').value)
        self._max_vy = float(self.get_parameter('max_vy').value)
        self._max_vz = float(self.get_parameter('max_vz').value)
        self._max_yaw = float(self.get_parameter('max_yaw_rate').value)
        self._slew_vx = float(self.get_parameter('slew_vx').value)
        self._slew_vy = float(self.get_parameter('slew_vy').value)
        self._slew_vz = float(self.get_parameter('slew_vz').value)
        self._slew_yaw = float(self.get_parameter('slew_yaw_rate').value)

        self._bbox_cfg = BBoxApproachConfig(
            image_width=float(self.get_parameter('image_width').value),
            image_height=float(self.get_parameter('image_height').value),
            target_u=float(self.get_parameter('target_u').value),
            target_v=float(self.get_parameter('target_v').value),
            desired_depth_m=float(self.get_parameter('desired_depth_m').value),
            stop_area_px=float(self.get_parameter('bbox_stop_area_px').value),
            k_u=float(self.get_parameter('k_u').value),
            k_v=float(self.get_parameter('k_v').value),
            k_depth=float(self.get_parameter('k_depth').value),
            k_area=float(self.get_parameter('k_area').value),
            k_yaw=float(self.get_parameter('k_yaw').value),
            use_vy=self._as_bool(self.get_parameter('bbox_use_vy').value),
            use_vz=self._as_bool(self.get_parameter('bbox_use_vz').value),
        )

        self._fx = float(self.get_parameter('fx_px').value)
        self._fy = float(self.get_parameter('fy_px').value)
        self._cx = float(self.get_parameter('cx_px').value)
        self._cy = float(self.get_parameter('cy_px').value)
        self._k_grasp_dist = float(self.get_parameter('k_grasp_distance').value)
        self._desired_grasp_dist = float(self.get_parameter('desired_grasp_distance_m').value)
        self._grasp_dist_mode = str(self.get_parameter('grasp_distance_mode').value).strip().lower()
        if self._grasp_dist_mode not in {'z', 'euclidean'}:
            self.get_logger().warn(f'Invalid grasp_distance_mode={self._grasp_dist_mode}, using z')
            self._grasp_dist_mode = 'z'
        self._require_grasp_pose = self._as_bool(self.get_parameter('require_grasp_pose').value)

        self._latest_obs: Optional[TargetObservation] = None
        self._latest_grasp: Optional[PoseStamped] = None
        self._last_obs_time = self.get_clock().now()
        self._last_grasp_time = self.get_clock().now()
        self._last_tick_time = self.get_clock().now()
        self._last_cmd = (0.0, 0.0, 0.0, 0.0)

        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self.create_subscription(TargetObservation, self._target_topic, self._obs_cb, 10)
        self.create_subscription(PoseStamped, self._grasp_pose_topic, self._grasp_cb, 10)

        self._mppi = self._build_mppi_if_enabled()
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / max(1e-3, rate_hz), self._tick)

        self.get_logger().info(
            f'visual_servo_controller_node mode={self._controller_mode} target={self._target_topic} cmd={self._cmd_topic}'
        )

    def _build_mppi_if_enabled(self):
        if self._controller_mode != 'mppi':
            return None
        if not _MPPI_AVAILABLE:
            self.get_logger().warn('MPPI requested but torch/core unavailable. Falling back to IBVS.')
            self._controller_mode = 'ibvs'
            return None

        lin_map = build_linear_map_from_axes(
            forward_axis='x',
            forward_sign=1.0,
            right_axis='y',
            right_sign=-1.0,
            down_axis='z',
            down_sign=-1.0,
        )
        cfg = {
            'use_cuda': False,
            'horizon_steps': 15,
            'num_samples': 256,
            'dt': 0.1,
            'lambda': 1.0,
            'noise_std': [0.15, 0.15, 0.15, 0.08],
            'control_min': [-self._max_vx, -self._max_vy, -self._max_vz, -self._max_yaw],
            'control_max': [self._max_vx, self._max_vy, self._max_vz, self._max_yaw],
            'w_pos': 10.0,
            'w_u_lin': 0.08,
            'w_u_ang': 0.12,
            'w_terminal': 12.0,
            'beta_risk': 0.0,
            'risk_mode': 'none',
            'risk_alpha': 0.9,
            'risk_agg': 'sum',
            'soft_violation_weight': 0.0,
            'p_des_raw': [self._cfg.desired_depth_m, 0.0, 0.0],
            'marker_update_sign': 1.0,
            'yaw_coupling_coeff': 0.0,
            'lin_map': lin_map.tolist(),
            'a_h': 1.0472,
            'a_v': 0.7854,
            'X_T': 0.13,
            'Y_T': 0.12,
            'depth_min': 0.1,
            'forward_axis_idx': axis_to_index('x'),
            'right_axis_idx': axis_to_index('y'),
            'down_axis_idx': axis_to_index('z'),
            'forward_sign': 1.0,
            'right_sign': 1.0,
            'down_sign': 1.0,
        }
        return MPPIController(cfg)

    def _obs_cb(self, msg: TargetObservation) -> None:
        self._latest_obs = msg
        self._last_obs_time = self.get_clock().now()

    def _grasp_cb(self, msg: PoseStamped) -> None:
        self._latest_grasp = msg
        self._last_grasp_time = self.get_clock().now()

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _publish_cmd(self, vx: float, vy: float, vz: float, yaw: float) -> None:
        vx = clamp(vx, self._max_vx)
        vy = clamp(vy, self._max_vy)
        vz = clamp(vz, self._max_vz)
        yaw = clamp(yaw, self._max_yaw)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._body_frame_id
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = yaw
        self._cmd_pub.publish(cmd)
        self._last_cmd = (vx, vy, vz, yaw)

    @staticmethod
    def _slew_axis(target: float, prev: float, limit_per_s: float, dt: float) -> float:
        if limit_per_s <= 0.0 or dt <= 0.0:
            return target
        delta_max = limit_per_s * dt
        delta = target - prev
        if delta > delta_max:
            return prev + delta_max
        if delta < -delta_max:
            return prev - delta_max
        return target

    def _publish_cmd_with_slew(self, vx: float, vy: float, vz: float, yaw: float, dt: float) -> None:
        pvx, pvy, pvz, pyaw = self._last_cmd
        svx = self._slew_axis(vx, pvx, self._slew_vx, dt)
        svy = self._slew_axis(vy, pvy, self._slew_vy, dt)
        svz = self._slew_axis(vz, pvz, self._slew_vz, dt)
        syaw = self._slew_axis(yaw, pyaw, self._slew_yaw, dt)
        self._publish_cmd(svx, svy, svz, syaw)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt_obs = (now - self._last_obs_time).nanoseconds * 1e-9
        dt_grasp = (now - self._last_grasp_time).nanoseconds * 1e-9
        dt_ctrl = max(1e-4, (now - self._last_tick_time).nanoseconds * 1e-9)
        self._last_tick_time = now

        if self._latest_obs is None or dt_obs > self._timeout_sec:
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            self._publish_status('NO_TARGET_TIMEOUT')
            return

        obs = Observation(
            u=float(self._latest_obs.u),
            v=float(self._latest_obs.v),
            w=float(self._latest_obs.w),
            h=float(self._latest_obs.h),
            depth_m=float(self._latest_obs.depth_m),
        )

        if self._controller_mode == 'mppi' and self._mppi is not None and math.isfinite(obs.depth_m):
            xr = (obs.u - self._cx) / max(1e-3, self._fx) * obs.depth_m
            yd = (obs.v - self._cy) / max(1e-3, self._fy) * obs.depth_m
            p_raw = [obs.depth_m, xr, yd]
            mppi_u = self._mppi.step(p_raw)
            self._publish_cmd_with_slew(float(mppi_u[0]), float(mppi_u[1]), float(mppi_u[2]), float(mppi_u[3]), dt_ctrl)
            self._publish_status('TRACKING_MPPI')
            return

        if self._controller_mode == 'bbox_approach':
            vx, vy, vz, yaw = compute_bbox_approach_command(obs, self._bbox_cfg)
            self._publish_cmd_with_slew(vx, vy, vz, yaw, dt_ctrl)
            self._publish_status('TRACKING_BBOX_APPROACH')
            return

        if self._controller_mode == 'bbox_grasp':
            # Keep bbox centering behavior for lateral/vertical/yaw channels.
            vx, vy, vz, yaw = compute_bbox_approach_command(obs, self._bbox_cfg)

            has_grasp = self._latest_grasp is not None and dt_grasp <= self._grasp_timeout_sec
            if has_grasp:
                gp = self._latest_grasp.pose.position
                if self._grasp_dist_mode == 'euclidean':
                    grasp_dist = math.sqrt(gp.x * gp.x + gp.y * gp.y + gp.z * gp.z)
                else:
                    grasp_dist = gp.z
                vx = self._k_grasp_dist * (grasp_dist - self._desired_grasp_dist)
                self._publish_cmd_with_slew(vx, vy, vz, yaw, dt_ctrl)
                self._publish_status('TRACKING_BBOX_GRASP')
                return

            if self._require_grasp_pose:
                self._publish_cmd_with_slew(0.0, 0.0, 0.0, 0.0, dt_ctrl)
                self._publish_status('NO_GRASP_TIMEOUT')
                return

            # Fallback: keep legacy bbox-only approach when grasp estimate is missing.
            self._publish_cmd_with_slew(vx, vy, vz, yaw, dt_ctrl)
            self._publish_status('TRACKING_BBOX_GRASP_FALLBACK')
            return

        vx, vy, vz, yaw = compute_ibvs_command(obs, self._cfg)
        self._publish_cmd_with_slew(vx, vy, vz, yaw, dt_ctrl)
        self._publish_status('TRACKING_IBVS')


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
