"""Visual servo controller node for the Phase 2 staged PID path.

Current behavior notes:
- Phase 2 controls body-frame x/y/yaw and can add z velocity after the
  horizontal gate is satisfied.
- `valid_target_observation` is only a tracking-continuity signal based on ROI
  consistency. It does not imply centered/aligned detection.
- `target_tracked` means the controller has a continuously trackable target-like
  observation. It does not imply horizontal lock or vertical-ready.
- Staged PID vertical defaults to target_point primary plus DVL fallback:
  `/sc2/visual_servo/target_point` (`PoseStamped`) is used while fresh; DVL
  altitude is used only when target_point is stale or unavailable. Fresh
  target_point always takes priority again when it recovers.
- `horizontal_aligned_now`, `vertical_enabled`, and `allow_z` gate z output.
  When `allow_z` is false, z velocity is forced to zero while horizontal control
  continues.
- Short stale gaps while `target_tracked` remains true decay the last horizontal
  command using `decay_rate`.
- The command adapter keeps the grapple IF and publishes both
  `TwistStamped` and `state_estimation/msg/Vector6`.
- `target_point.pose.position.z` is expected to be target-relative range in
  meters. Before hardware testing, still verify the resulting positive-z command
  convention against the vehicle frame.

Not implemented in this file yet: search scan/recovery. The staged PID path
implements a terminal completion gate for the external operation state machine.
"""

from collections import deque
from typing import Optional
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3
from vision_msgs.msg import Detection2D
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import CameraInfo
from sc2_visual_servo_interfaces.msg import TargetObservation
from enum import Enum

from state_estimation.msg import Vector6
try:
    from marine_acoustic_msgs.msg import Dvl
    _HAS_MARINE_ACOUSTIC_DVL = True
except ImportError:
    Dvl = None
    _HAS_MARINE_ACOUSTIC_DVL = False

from .ibvs.controller_math import (
    clamp,
    is_in_roi
)
from .hold_maintain_core import (
    HoldVelocityDampingConfig,
    HoldVelocityMeasurement,
    compute_hold_velocity_damping,
    should_enter_hold_maintain,
)
from .pid import PIDController, PIDGains

class VisualServoControllerNode(Node):
    
    # state machine
    class ControllerState(Enum):
        BREAK = 1
        MANUAL = 2          # TODO:Implement Manual Mode as Control via the Joystick
        VISUAL = 3
        DVL = 4 
        COMPLETE = 5

    class StagedPIDState(Enum):
        HORIZONTAL = 'HORIZONTAL'
        HORIZONTAL_AND_VERTICAL = 'HORIZONTAL_AND_VERTICAL'
        NO_TARGET = 'NO_TARGET'
    
    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def __init__(self):
        super().__init__('visual_servo_controller_node')

        # relevant data topics
        self.declare_parameter('target_topic', '/sc2/visual_servo/target_obs')
        self.declare_parameter('use_target_observation', False)
        self.declare_parameter('target_point_topic', '/sc2/visual_servo/target_point')
        self.declare_parameter('target_bounding_box', '/best_detection')
        self.declare_parameter('cmd_topic', '/mavros/setpoint_velocity/cmd_vel') #mavros commanded velocity topic taken from the mavros docs
        self.declare_parameter('cmd_topic_vec_6', '/sc2/setpoint_velocity/cmd_vel_vec6') #the 
        self.declare_parameter('vel_measurement_topic', '/mavros/local_position/velocity_body')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('status_topic', '/sc2/visual_servo/status')
        self.declare_parameter('target_locked_state', '/sc2/visual_servo/target_locked')
        self.declare_parameter('centroid_error_topic', '/sc2/visual_servo/centroid_error')
        self.declare_parameter('dvl_altitude_topic', '/waterlinked_dvl_driver/velocity_report')
        self.declare_parameter('mavros_altitude_topic','/mavros/global_position/rel_alt')
        self.declare_parameter('mavros_depth_topic', '/mavros/global_position/rel_alt')
        
        # relevant frames for transformations
        self.declare_parameter('body_frame_id', 'base_link')                # frame used for direct body-frame commands

        # controller parameters 
        self.declare_parameter('timeout_ms', 500)
        self.declare_parameter('target_point_timeout_ms', 500)
        self.declare_parameter('rate_hz', 60.0)
        self.declare_parameter('controller_mode', 'staged_pid')
        self.declare_parameter('calibrated', True)
        self.declare_parameter('verbose', True)
        self.declare_parameter('enable_dvl', False)
        self.declare_parameter('yaw_control', False)    #turn yaw control on/off
        self.declare_parameter('depth_estimation_mode', 'RGBD') # choose from: MAVROS, DVL or RGBD

        # image data parameters
        self.declare_parameter('image_width', 1280.0)
        self.declare_parameter('image_height', 640.0)
        self.declare_parameter('target_u', 640.0)
        self.declare_parameter('target_v', 360.0)

        # vertical control parameters
        self.declare_parameter('desired_depth_m', 1.0)
        self.declare_parameter('desired_target_distance_m', 0.6)
        self.declare_parameter('vertical_gain', 1.0)

        # controller parameters
        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_vz', 0.3)
        self.declare_parameter('max_yaw_rate', 0.4)
        self.declare_parameter('slew_vx', 2.0)
        self.declare_parameter('slew_vy', 2.0)
        self.declare_parameter('slew_vz', 2.0)
        self.declare_parameter('slew_yaw_rate', 2.0)

        # camera parameters
        self.declare_parameter('fx_px', 600.0)
        self.declare_parameter('fy_px', 600.0)
        self.declare_parameter('cx_px', 640.0)
        self.declare_parameter('cy_px', 360.0)

        # staged PID parameters. Phase 2 adds target_pose-sourced vertical PID
        # while preserving Phase 1 horizontal behavior in HORIZONTAL.
        #
        # Vertical gate tolerance rationale:
        # With HFOV ~= 66 deg and the controller bbox image width set to 640 px,
        # at 0.5 m range 1 px is approximately 1 mm laterally. Therefore
        # 25 px ~= 2.5 cm, 150 px ~= 15 cm, and 200 px ~= 20 cm. The Phase 2
        # vertical gate uses 150 px on both u and v: slightly stricter than a
        # 0.2 m lateral-offset allowance at 0.5 m while avoiding the previous
        # 25 px near-perfect-centering assumption.
        self.declare_parameter('staged_pid_xy_kp', 0.0025)
        self.declare_parameter('staged_pid_xy_ki', 0.0)
        self.declare_parameter('staged_pid_xy_kd', 0.0008)
        self.declare_parameter('staged_pid_xy_integral_limit', 200.0)
        self.declare_parameter('staged_pid_z_kp', 0.8)
        self.declare_parameter('staged_pid_z_ki', 0.0)
        self.declare_parameter('staged_pid_z_kd', 0.10)
        self.declare_parameter('staged_pid_z_integral_limit', 1.0)
        self.declare_parameter('staged_pid_horizontal_tolerance_px', 150.0)
        self.declare_parameter('staged_pid_horizontal_tolerance_x_px', 150.0)
        self.declare_parameter('staged_pid_horizontal_tolerance_y_px', 150.0)
        self.declare_parameter('staged_pid_vertical_tolerance_px', 150.0)
        self.declare_parameter('staged_pid_distance_tolerance_m', 0.08)
        self.declare_parameter('staged_pid_vertical_gate_window_ticks', 10)
        self.declare_parameter('staged_pid_vertical_enter_ratio', 0.8)
        self.declare_parameter('staged_pid_vertical_exit_ratio', 0.4)
        self.declare_parameter('staged_pid_vertical_enter_hold_ticks', 2)
        self.declare_parameter('staged_pid_vertical_exit_hold_ticks', 2)
        self.declare_parameter('staged_pid_enable_yaw', False)
        self.declare_parameter('desired_vertical_distance_m', 0.6)
        self.declare_parameter('staged_pid_z_source', 'target_point_primary_dvl_fallback')
        self.declare_parameter('staged_pid_hold_zero_distance_m', 0.5)
        self.declare_parameter('vs_control_finish_topic', '/sc2/visual_servo/vs_control_finish')
        self.declare_parameter('staged_pid_completion_enable', True)
        self.declare_parameter('staged_pid_completion_window_ticks', 10)
        self.declare_parameter('staged_pid_completion_enter_ratio', 0.8)
        self.declare_parameter('staged_pid_completion_latch', True)
        self.declare_parameter('hold_maintain_enable', True)
        self.declare_parameter('hold_maintain_z_kp_scale', 0.5)
        self.declare_parameter('hold_maintain_velocity_damping_enable', True)
        self.declare_parameter('hold_maintain_xy_damping', 0.4)
        self.declare_parameter('hold_maintain_yaw_damping', 0.3)
        self.declare_parameter('hold_maintain_velocity_timeout_ms', 500)
        self.declare_parameter('tracked_enter_window_obs', 10)
        self.declare_parameter('tracked_enter_min_obs', 2)
        self.declare_parameter('tracked_exit_no_obs_sec', 0.6)
        
        # IBVS control parameters
        self.declare_parameter('decay_rate', 0.95)

        # extract data topics
        self._target_bbox_topic = str(self.get_parameter('target_bounding_box').value)
        self._target_obs_topic = str(self.get_parameter('target_topic').value)
        self._use_target_observation = self._as_bool(self.get_parameter('use_target_observation').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._sc2_cmd_topic_vec_6 = str(self.get_parameter('cmd_topic_vec_6').value)
        self._velocity_measurement_topic = str(self.get_parameter('vel_measurement_topic').value)
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._target_locked_topic = str(self.get_parameter('target_locked_state').value)
        self._centroid_error_topic = str(self.get_parameter('centroid_error_topic').value)
        self._vs_control_finish_topic = str(self.get_parameter('vs_control_finish_topic').value)

        # extract frame IDs
        self._body_frame_id = str(self.get_parameter('body_frame_id').value)

        # extract parameters
        self._timeout_sec = float(self.get_parameter('timeout_ms').value) * 1e-3
        self._target_point_timeout_sec = float(self.get_parameter('target_point_timeout_ms').value) * 1e-3
        self._controller_mode = str(self.get_parameter('controller_mode').value).strip().lower()
        if self._controller_mode != 'staged_pid':
            raise RuntimeError(
                f'Unsupported controller_mode={self._controller_mode!r}; '
                'this node now supports only staged_pid'
            )
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.enable_dvl = self.get_parameter('enable_dvl').get_parameter_value().bool_value
        self._yaw_control_switch = self.get_parameter('yaw_control').get_parameter_value().bool_value
        self._calibrated = self.get_parameter('calibrated').get_parameter_value().bool_value
        self._target_point_topic = str(self.get_parameter('target_point_topic').value)
        self._depth_mode = str(self.get_parameter('depth_estimation_mode').get_parameter_value().string_value)
        mavros_depth_topic = str(self.get_parameter('mavros_depth_topic').value)
        mavros_altitude_topic = str(self.get_parameter('mavros_altitude_topic').value)
        self._mavros_altitude_topic = mavros_altitude_topic
        if mavros_depth_topic != '/mavros/global_position/rel_alt':
            self._mavros_altitude_topic = mavros_depth_topic

        # which depth estimation mode to use for IBVS
        match self._depth_mode:
            case 'RGBD':
                self._target_pose_topic = self._target_point_topic
            case 'DVL':
                self._target_pose_topic = str(self.get_parameter('dvl_altitude_topic').value)
            case 'MAVROS':
                self._target_pose_topic = self._mavros_altitude_topic

        self._max_vx = float(self.get_parameter('max_vx').value)
        self._max_vy = float(self.get_parameter('max_vy').value)
        self._max_vz = float(self.get_parameter('max_vz').value)
        self._max_yaw = float(self.get_parameter('max_yaw_rate').value)
        self._slew_vx = float(self.get_parameter('slew_vx').value)
        self._slew_vy = float(self.get_parameter('slew_vy').value)
        self._slew_vz = float(self.get_parameter('slew_vz').value)
        self._slew_yaw = float(self.get_parameter('slew_yaw_rate').value)

        # IBVS paramters
        self.decay_rate_val = float(self.get_parameter('decay_rate').value)

        self.calibrated = bool(self.get_parameter('calibrated').value)
        self._fx = float(self.get_parameter('fx_px').value)
        self._fy = float(self.get_parameter('fy_px').value)
        self._cx = float(self.get_parameter('cx_px').value)
        self._cy = float(self.get_parameter('cy_px').value)
        self.image_height = float(self.get_parameter('image_height').value)
        self.image_width = float(self.get_parameter('image_width').value)
        self._target_u = float(self.get_parameter('target_u').value)
        self._target_v = float(self.get_parameter('target_v').value)
        self._desired_target_dist = float(self.get_parameter('desired_target_distance_m').value)
        self._desired_vertical_dist = float(self.get_parameter('desired_vertical_distance_m').value)
        self._staged_pid_z_source = str(self.get_parameter('staged_pid_z_source').value).strip().lower()
        if self._staged_pid_z_source in {'target_pose', 'target_point'}:
            self._staged_pid_z_source = 'target_point'
        elif self._staged_pid_z_source in {'target_point_primary_dvl_fallback', 'target_pose_primary_dvl_fallback'}:
            self._staged_pid_z_source = 'target_point_primary_dvl_fallback'
        elif self._staged_pid_z_source in {'dvl', 'dvl_fallback'}:
            self._staged_pid_z_source = 'dvl'
        else:
            self.get_logger().warn(
                f'Invalid staged_pid_z_source={self._staged_pid_z_source}; using target_point_primary_dvl_fallback'
            )
            self._staged_pid_z_source = 'target_point_primary_dvl_fallback'

        self._latest_obs: Optional[Detection2D] = None
        self._latest_target_observation: Optional[TargetObservation] = None
        self._target_observation_valid = False
        self._target_observation_state = 'LOST'
        self._target_observation_reason = 'not_received'
        self._latest_target_point: Optional[PoseStamped] = None

        self.first_execution = True
        self._last_obs_time = self.get_clock().now()
        self._last_target_point_time = self.get_clock().now()
        self._last_depth_time = None
        self._last_tick_time = self.get_clock().now()
        self._last_cmd = (0.0, 0.0, 0.0, 0.0)

        self._xy_pid_x = PIDController(PIDGains(
            kp=float(self.get_parameter('staged_pid_xy_kp').value),
            ki=float(self.get_parameter('staged_pid_xy_ki').value),
            kd=float(self.get_parameter('staged_pid_xy_kd').value),
            integral_limit=float(self.get_parameter('staged_pid_xy_integral_limit').value),
            output_limit=self._max_vx,
        ))
        self._xy_pid_y = PIDController(PIDGains(
            kp=float(self.get_parameter('staged_pid_xy_kp').value)/2.0,
            ki=float(self.get_parameter('staged_pid_xy_ki').value)/2.0,
            kd=float(self.get_parameter('staged_pid_xy_kd').value)/2.0,
            integral_limit=float(self.get_parameter('staged_pid_xy_integral_limit').value),
            output_limit=self._max_vy,
        ))
        self._z_pid = PIDController(PIDGains(
            kp=float(self.get_parameter('staged_pid_z_kp').value),
            ki=float(self.get_parameter('staged_pid_z_ki').value),
            kd=float(self.get_parameter('staged_pid_z_kd').value),
            integral_limit=float(self.get_parameter('staged_pid_z_integral_limit').value),
            output_limit=self._max_vz,
        ))
        self._staged_pid_enable_yaw = self._as_bool(self.get_parameter('staged_pid_enable_yaw').value)
        default_vertical_gate_tol_px = 150.0
        horizontal_tol_px = float(self.get_parameter('staged_pid_horizontal_tolerance_px').value)
        horizontal_tol_x_px = float(self.get_parameter('staged_pid_horizontal_tolerance_x_px').value)
        horizontal_tol_y_px = float(self.get_parameter('staged_pid_horizontal_tolerance_y_px').value)
        vertical_tol_px = float(self.get_parameter('staged_pid_vertical_tolerance_px').value)
        if horizontal_tol_x_px == default_vertical_gate_tol_px and horizontal_tol_px != default_vertical_gate_tol_px:
            horizontal_tol_x_px = horizontal_tol_px
        if horizontal_tol_y_px == default_vertical_gate_tol_px and horizontal_tol_px != default_vertical_gate_tol_px:
            horizontal_tol_y_px = horizontal_tol_px
        if horizontal_tol_y_px == default_vertical_gate_tol_px and vertical_tol_px != default_vertical_gate_tol_px:
            self.get_logger().warn(
                'staged_pid_vertical_tolerance_px is deprecated for vertical gate on/off; '
                'using it as a y-axis tolerance alias because staged_pid_horizontal_tolerance_y_px '
                'was left at its default.'
            )
            horizontal_tol_y_px = vertical_tol_px
        self._staged_pid_horizontal_tol_x_px = horizontal_tol_x_px
        self._staged_pid_horizontal_tol_y_px = horizontal_tol_y_px
        self._staged_pid_vertical_tol_px = horizontal_tol_y_px
        self._staged_pid_distance_tol_m = float(self.get_parameter('staged_pid_distance_tolerance_m').value)
        self._staged_pid_vertical_gate_window_ticks = max(
            1, int(self.get_parameter('staged_pid_vertical_gate_window_ticks').value)
        )
        self._staged_pid_vertical_enter_ratio = float(self.get_parameter('staged_pid_vertical_enter_ratio').value)
        self._staged_pid_vertical_exit_ratio = float(self.get_parameter('staged_pid_vertical_exit_ratio').value)
        self._staged_pid_vertical_enter_hold_ticks = max(
            1, int(self.get_parameter('staged_pid_vertical_enter_hold_ticks').value)
        )
        self._staged_pid_vertical_exit_hold_ticks = max(
            1, int(self.get_parameter('staged_pid_vertical_exit_hold_ticks').value)
        )
        self._staged_pid_hold_zero_distance_m = float(
            self.get_parameter('staged_pid_hold_zero_distance_m').value
        )
        self._staged_pid_completion_enable = self._as_bool(
            self.get_parameter('staged_pid_completion_enable').value
        )
        self._staged_pid_completion_window_ticks = max(
            1, int(self.get_parameter('staged_pid_completion_window_ticks').value)
        )
        self._staged_pid_completion_enter_ratio = min(
            1.0,
            max(0.0, float(self.get_parameter('staged_pid_completion_enter_ratio').value)),
        )
        self._staged_pid_completion_latch = self._as_bool(
            self.get_parameter('staged_pid_completion_latch').value
        )
        self._hold_maintain_enable = self._as_bool(self.get_parameter('hold_maintain_enable').value)
        self._hold_maintain_z_kp_scale = max(
            0.0,
            float(self.get_parameter('hold_maintain_z_kp_scale').value),
        )
        self._hold_maintain_velocity_damping_enable = self._as_bool(
            self.get_parameter('hold_maintain_velocity_damping_enable').value
        )
        self._hold_maintain_xy_damping = max(
            0.0,
            float(self.get_parameter('hold_maintain_xy_damping').value),
        )
        self._hold_maintain_yaw_damping = max(
            0.0,
            float(self.get_parameter('hold_maintain_yaw_damping').value),
        )
        self._hold_maintain_velocity_timeout_sec = (
            float(self.get_parameter('hold_maintain_velocity_timeout_ms').value) * 1e-3
        )
        self._tracked_enter_window_obs = max(
            1, int(self.get_parameter('tracked_enter_window_obs').value)
        )
        self._tracked_enter_min_obs = max(
            1, int(self.get_parameter('tracked_enter_min_obs').value)
        )
        self._tracked_exit_no_obs_sec = float(self.get_parameter('tracked_exit_no_obs_sec').value)
        self._target_tracked = False
        self._latest_detection_time = None
        self._last_valid_target_observation_time = None
        self._last_accepted_target_centroid: Optional[np.ndarray] = None
        self._has_marine_acoustic_dvl = _HAS_MARINE_ACOUSTIC_DVL
        self._tracking_valid_observation_history = deque(maxlen=self._tracked_enter_window_obs)
        #self._staged_pid_state = self.StagedPIDState.NO_TARGET
        self._staged_pid_phase = 'horizontal_only'
        self._staged_pid_recent_horizontal_ok = deque(
            maxlen=self._staged_pid_vertical_gate_window_ticks
        )
        self._staged_pid_recent_completion_ok = deque(
            maxlen=self._staged_pid_completion_window_ticks
        )
        self._vs_control_finished = False
        self._staged_pid_vertical_enabled = False
        self._staged_pid_enter_counter = 0
        self._staged_pid_exit_counter = 0
        self._staged_pid_horizontal_ok_ratio = 0.0
        self._staged_pid_vertical_scale = 0.0
        self._staged_pid_last_horizontal_aligned = False
        self._staged_pid_last_allow_z = False
        self._staged_pid_last_z_fresh = False
        self._staged_pid_last_z_meas: Optional[float] = None
        self._staged_pid_last_z_source = 'none'

        # Target tracking and command state.
        # target paratmers
        # first define the corners of a sqaure around the center of the image
        # TODO: validate whether these target corners suffice 
        side_length = 100
        h = side_length / 2.0
        cx, cy = self.image_height / 2.0, self.image_height / 2.0

        self.center_box_corners = np.array([
            [cx - h, cy - h],  # Top-Left
            [cx + h, cy - h],  # Top-Right
            [cx + h, cy + h],  # Bottom-Right
            [cx - h, cy + h]   # Bottom-Left
        ], dtype=np.float32)

        #self.target_centroid = np.mean(self.center_box_corners, axis=1)
        self.target_centroid = [cx, cy]
        self._staged_pid_target_centroid = np.array([self._target_u, self._target_v], dtype=np.float32)
        self.latest_centroid = [0.0, 0.0]

        self.roi_w = 50

        # define the minimum needed detections and the detection counter

        # IBVS parameters
        self.v_last_valid = np.zeros(6)
        self.decay_rate = self.decay_rate_val  # Drops 5% of speed per "blind" frame
        self.latest_velocity_measurement = np.zeros(6)  # a variable to store the incoming velocity measurements from mavros/pixhawk
        self._last_velocity_measurement_time = None

        # DVL control parameters
        self._latest_depth_measurement = 3.0
        self.dist_target = float(self.get_parameter('desired_target_distance_m').value) # desired vertical distance to target     
        self.kz = float(self.get_parameter('vertical_gain').value) # vertical gain

        # uncalibrated visual servoing

        # data publishers
        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_topic, 10)                            # the actual commanded velocity in the Pixhawk base_link frame        
        self._sc2_cmd_pub_vec_6 = self.create_publisher(Vector6, self._sc2_cmd_topic_vec_6, 10)   
        
        # status and debug publishers
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._target_locked_pub = self.create_publisher(String, self._target_locked_topic, 10)
        self._vs_control_finish_pub = self.create_publisher(Bool, self._vs_control_finish_topic, 10)
        self._centroid_error_publisher = self.create_publisher(Vector3, self._centroid_error_topic, 10)

        self.create_subscription(Detection2D, self._target_bbox_topic, self._detection_cb, 10)
        if self._use_target_observation:
            self.create_subscription(TargetObservation, self._target_obs_topic, self._target_observation_cb, 10)
        self.create_subscription(CameraInfo, self._camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(TwistStamped, self._velocity_measurement_topic, self._vel_measurement_cb, 10)

        self.create_subscription(PoseStamped, self._target_point_topic, self._target_pose_cb, 10)
        if self._depth_mode == 'DVL':
            if self._has_marine_acoustic_dvl:
                self.create_subscription(
                    Dvl,
                    str(self.get_parameter('dvl_altitude_topic').value),
                    self._dvl_altitude_cb,
                    10,
                )
            else:
                self.get_logger().warn(
                    'marine_acoustic_msgs.msg.Dvl is unavailable in this environment; '
                    'staged PID will run without /waterlinked_dvl_driver/velocity_report depth input.'
                )
        elif self._depth_mode == 'MAVROS':
            self.create_subscription(
                Float64,
                self._mavros_altitude_topic,
                self._mavros_altitude_cb,
                10,
            )
        
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / max(1e-3, rate_hz), self._tick)

        self.current_state = self.ControllerState.BREAK
        self._publish_status(str(self.current_state.name))
        self._publish_vs_control_finish(False)

        self.get_logger().info(
            f'visual_servo_controller_node mode={self._controller_mode} '
            f'target_bbox={self._target_bbox_topic} target_obs={self._target_obs_topic if self._use_target_observation else "<disabled>"} '
            f'cmd={self._cmd_topic} control=always_enabled'
        )

    def _reset_staged_pid_state(self) -> None:
        self._xy_pid_x.reset()
        self._xy_pid_y.reset()
        self._reset_vertical_gate()
        self._clear_staged_pid_completion()

    def _clear_staged_pid_completion(self) -> None:
        self._staged_pid_recent_completion_ok.clear()
        self._vs_control_finished = False

    def _reset_vertical_gate(self) -> None:
        self._z_pid.reset()
        self._staged_pid_recent_horizontal_ok.clear()
        self._staged_pid_vertical_enabled = False
        self._staged_pid_enter_counter = 0
        self._staged_pid_exit_counter = 0
        self._staged_pid_horizontal_ok_ratio = 0.0
        self._staged_pid_vertical_scale = 0.0
        self._staged_pid_last_horizontal_aligned = False
        self._staged_pid_last_allow_z = False
        self._staged_pid_last_z_fresh = False
        self._staged_pid_last_z_meas = None
        self._staged_pid_last_z_source = 'none'
        self._staged_pid_phase = 'horizontal_only'

    def _set_target_tracked(self, tracked: bool) -> None:
        if tracked == self._target_tracked:
            return
        self._target_tracked = tracked
        if tracked:
            #self._staged_pid_state = self.StagedPIDState.HORIZONTAL
            #print(f"setting `tracked` to True. `self._staged_pid_state = self.StagedPIDState.HORIZONTAL`")
            self._reset_vertical_gate()
        else:
            self._tracking_valid_observation_history.clear()
            self._last_accepted_target_centroid = None
            #self._staged_pid_state = self.StagedPIDState.NO_TARGET
            #print(f"Not tracked. Setting `self._staged_pid_state = sself.StagedPIDState.NO_TARGET")
            self._reset_staged_pid_state()

    def _is_timestamp_fresh(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        return ((self.get_clock().now() - stamp).nanoseconds * 1e-9) <= timeout_sec

    def _update_target_tracking_from_detection(self, now) -> bool:
        if not hasattr(self, 'latest_centroid'):
            return False

        centroid = np.array(self.latest_centroid, dtype=np.float32)
        roi_consistent = (
            self._last_accepted_target_centroid is None or
            is_in_roi(centroid[0], centroid[1], self._last_accepted_target_centroid, self.roi_w)
        )
        print(f"roi_consistent: {roi_consistent}")
        if roi_consistent == False:
            roi_consistent = True
            print("actually roi_consistent should be False, but treating it as True for making debugging easier")
        valid_target_observation = bool(roi_consistent)
        self._tracking_valid_observation_history.append(valid_target_observation)

        if valid_target_observation:
            self._last_accepted_target_centroid = centroid
            self._last_valid_target_observation_time = now

        if (
            not self._target_tracked and
            sum(self._tracking_valid_observation_history) >= self._tracked_enter_min_obs
        ):
            self._set_target_tracked(True)
            print(f"sum(self._tracking_valid_observation_history) >= self._tracked_enter_min_obs: {sum(self._tracking_valid_observation_history) >= self._tracked_enter_min_obs}")
            print("setting `self._set_target_tracked(True)`")

        return valid_target_observation

    def _update_target_tracked_from_tick(self) -> None:
        if (
            self._target_tracked and
            not self._is_timestamp_fresh(self._last_valid_target_observation_time, self._tracked_exit_no_obs_sec)
        ):
            #self._set_target_tracked(False)
            print("originally setting self._set_target_tracked(False) but now setting off")
            pass

    def _tracked_horizontal_stale_command(self) -> tuple[float, float, float, float]:
        return (
            float(self.v_last_valid[0] * self.decay_rate),
            float(self.v_last_valid[1] * self.decay_rate),
            0.0,
            float(self.v_last_valid[5] * self.decay_rate),
        )

    def _compute_staged_pid_horizontal_command(self, dt_ctrl: float) -> tuple[float, float, float, float, str]:
        centroid_err_x = float(self.latest_centroid[0] - self._staged_pid_target_centroid[0])
        centroid_err_y = float(self.latest_centroid[1] - self._staged_pid_target_centroid[1])

        vel_x_im = self._xy_pid_x.update(centroid_err_x, dt_ctrl)
        vel_y_im = self._xy_pid_y.update(centroid_err_y, dt_ctrl)

        cmd_vx_body = vel_x_im
        cmd_vy_body = -vel_y_im
        yaw = 0.0
        if self._staged_pid_enable_yaw:
            yaw = clamp(
                0.8 * (centroid_err_x / max(1.0, self.image_width * 0.5)),
                self._max_yaw,
            )

        return cmd_vx_body, cmd_vy_body, 0.0, yaw, 'TRACKING_STAGED_PID_HORIZONTAL'

    def _update_staged_pid_vertical_gate(self, obs_fresh: bool) -> tuple[bool, float]:
        horizontal_aligned = False
        if obs_fresh and hasattr(self, 'latest_centroid'):
            centroid_err_x = float(self.latest_centroid[0] - self._staged_pid_target_centroid[0])
            centroid_err_y = float(self.latest_centroid[1] - self._staged_pid_target_centroid[1])
            horizontal_aligned = (
                abs(centroid_err_x) <= self._staged_pid_horizontal_tol_x_px and
                abs(centroid_err_y) <= self._staged_pid_horizontal_tol_y_px
            )

        self._staged_pid_recent_horizontal_ok.append(horizontal_aligned)
        ok_count = sum(self._staged_pid_recent_horizontal_ok)
        horizontal_ok_ratio = ok_count / float(self._staged_pid_vertical_gate_window_ticks)

        if horizontal_ok_ratio >= self._staged_pid_vertical_enter_ratio:
            self._staged_pid_enter_counter += 1
        else:
            self._staged_pid_enter_counter = 0

        if horizontal_ok_ratio <= self._staged_pid_vertical_exit_ratio:
            self._staged_pid_exit_counter += 1
        else:
            self._staged_pid_exit_counter = 0

        if (
            not self._staged_pid_vertical_enabled and
            self._staged_pid_enter_counter >= self._staged_pid_vertical_enter_hold_ticks
        ):
            self._staged_pid_vertical_enabled = True
            self._staged_pid_exit_counter = 0
        elif (
            self._staged_pid_vertical_enabled and
            self._staged_pid_exit_counter >= self._staged_pid_vertical_exit_hold_ticks
        ):
            self._staged_pid_vertical_enabled = False
            self._staged_pid_enter_counter = 0
            self._z_pid.reset()

        self._staged_pid_phase = (
            'vertical_enabled' if self._staged_pid_vertical_enabled else 'horizontal_only'
        )
        self._staged_pid_horizontal_ok_ratio = horizontal_ok_ratio
        self._staged_pid_vertical_scale = horizontal_ok_ratio if self._staged_pid_vertical_enabled else 0.0
        self._staged_pid_last_horizontal_aligned = horizontal_aligned
        return horizontal_aligned, horizontal_ok_ratio

    def _is_staged_pid_vertical_gate_aligned(self) -> bool:
        if not hasattr(self, 'latest_centroid'):
            return False
        centroid_err_x = float(self.latest_centroid[0] - self._staged_pid_target_centroid[0])
        centroid_err_y = float(self.latest_centroid[1] - self._staged_pid_target_centroid[1])
        return (
            abs(centroid_err_x) <= self._staged_pid_horizontal_tol_x_px and
            abs(centroid_err_y) <= self._staged_pid_horizontal_tol_y_px
        )

    def _resolve_staged_pid_vertical_measurement(self) -> tuple[Optional[float], str, bool]:
        target_point_fresh = (
            self._latest_target_point is not None and
            self._is_timestamp_fresh(self._last_target_point_time, self._target_point_timeout_sec)
        )
        dvl_fresh = (
            self._latest_depth_measurement is not None and
            self._is_timestamp_fresh(self._last_depth_time, self._target_point_timeout_sec)
        )

        if self._staged_pid_z_source in {'target_point', 'target_point_primary_dvl_fallback'}:
            if target_point_fresh:
                return float(self._latest_target_point.pose.position.z), 'target_point', True
            if self._staged_pid_z_source == 'target_point_primary_dvl_fallback' and dvl_fresh:
                return float(self._latest_depth_measurement), 'dvl_fallback', True
            return None, 'none', False

        if self._staged_pid_z_source == 'dvl':
            if dvl_fresh:
                return float(self._latest_depth_measurement), 'dvl', True
            return None, 'none', False

        return None, 'none', False

    def _update_staged_pid_completion_gate(
        self,
        *,
        obs_fresh: bool,
        z_meas: Optional[float],
        z_fresh: bool,
    ) -> bool:
        """Return true after bbox and distance have remained ready long enough.

        This is a state-machine handoff condition for the staged PID path. It is
        deliberately stricter than target tracking: tracking only means that the
        selected observation is continuous, whereas completion requires both image
        centering and vertical-distance convergence.
        """
        if not self._staged_pid_completion_enable:
            self._clear_staged_pid_completion()
            return False

        if self._staged_pid_completion_latch and self._vs_control_finished:
            return True

        selected_valid_ok = True
        if self._use_target_observation:
            selected_valid_ok = bool(self._target_observation_valid)

        horizontal_ok = self._is_staged_pid_vertical_gate_aligned()
        distance_ok = (
            z_fresh
            and z_meas is not None
            and abs(float(z_meas) - self._desired_vertical_dist) <= self._staged_pid_distance_tol_m
        )
        completion_ok = (
            self._target_tracked
            and obs_fresh
            and selected_valid_ok
            and horizontal_ok
            and distance_ok
        )

        self._staged_pid_recent_completion_ok.append(bool(completion_ok))
        if len(self._staged_pid_recent_completion_ok) < self._staged_pid_completion_window_ticks:
            return False

        ready_ratio = sum(self._staged_pid_recent_completion_ok) / float(
            self._staged_pid_completion_window_ticks
        )
        return ready_ratio >= self._staged_pid_completion_enter_ratio

    def _compute_staged_pid_command(
        self,
        obs_fresh: bool,
        dt_ctrl: float,
    ) -> tuple[float, float, float, float, str, bool, Optional[float], bool, str]:
        vx_body, vy_body, _, yaw, _ = self._compute_staged_pid_horizontal_command(dt_ctrl)
        horizontal_aligned_now, _ = self._update_staged_pid_vertical_gate(obs_fresh)
        z_meas, z_source, z_fresh = self._resolve_staged_pid_vertical_measurement()
        allow_z = (
            self._staged_pid_vertical_enabled and
            horizontal_aligned_now and
            z_fresh and
            z_meas is not None
        )

        vz_body = 0.0
        if allow_z:
            err_z_m = float(z_meas - self._desired_vertical_dist)
            vz_raw = self._z_pid.update(err_z_m, dt_ctrl)
            vz_body = self._staged_pid_vertical_scale * vz_raw
            aligned = (
                self._is_staged_pid_vertical_gate_aligned() and
                abs(err_z_m) <= self._staged_pid_distance_tol_m
            )
            # Phase 2 only exposes the target-distance terminal condition as
            # status. It does not enter a hold-zero state or change the command.
            if z_meas <= self._staged_pid_hold_zero_distance_m:
                status = f'TRACKING_STAGED_PID_TARGET_DISTANCE_REACHED_{z_source.upper()}'
            else:
                source_suffix = z_source.upper()
                status = (
                    f'TRACKING_STAGED_PID_ALIGNED_{source_suffix}'
                    if aligned else
                    f'TRACKING_STAGED_PID_VERTICAL_{source_suffix}'
                )
            #self._staged_pid_state = self.StagedPIDState.HORIZONTAL_AND_VERTICAL
            #print(f"`self._staged_pid_state = self.StagedPIDState.HORIZONTAL_AND_VERTICAL")
        else:
            self._z_pid.reset()
            if self._staged_pid_vertical_enabled and horizontal_aligned_now and not z_fresh:
                status = 'TRACKING_STAGED_PID_HORIZONTAL_Z_STALE'
            elif horizontal_aligned_now:
                status = 'TRACKING_STAGED_PID_HORIZONTAL_READY'
            else:
                status = 'TRACKING_STAGED_PID_HORIZONTAL'
            #self._staged_pid_state = self.StagedPIDState.HORIZONTAL
            #print(f"`self._staged_pid_state = self.StagedPIDState.HORIZONTAL")

        self._staged_pid_last_allow_z = allow_z
        self._staged_pid_last_z_fresh = z_fresh
        self._staged_pid_last_z_meas = z_meas
        self._staged_pid_last_z_source = z_source
        return vx_body, vy_body, vz_body, yaw, status, allow_z, z_meas, z_fresh, z_source

    def _compute_hold_maintain_command(self, dt_ctrl: float) -> tuple[float, float, float, float, str]:
        if not self._hold_maintain_enable:
            self._z_pid.reset()
            return 0.0, 0.0, 0.0, 0.0, 'HOLD_MAINTAIN_DISABLED'

        velocity_fresh = (
            self._last_velocity_measurement_time is not None and
            self._is_timestamp_fresh(
                self._last_velocity_measurement_time,
                self._hold_maintain_velocity_timeout_sec,
            )
        )
        vx_body = 0.0
        vy_body = 0.0
        yaw = 0.0
        velocity_status = 'NO_VELOCITY'
        if self._hold_maintain_velocity_damping_enable and velocity_fresh:
            vx_body, vy_body, yaw = compute_hold_velocity_damping(
                HoldVelocityMeasurement(
                    vx=float(self.latest_velocity_measurement[0]),
                    vy=float(self.latest_velocity_measurement[1]),
                    yaw_rate=float(self.latest_velocity_measurement[5]),
                ),
                HoldVelocityDampingConfig(
                    xy_damping=self._hold_maintain_xy_damping,
                    yaw_damping=self._hold_maintain_yaw_damping,
                    max_vx=self._max_vx,
                    max_vy=self._max_vy,
                    max_yaw=self._max_yaw,
                ),
            )
            velocity_status = 'VELOCITY_DAMPING'
        elif not self._hold_maintain_velocity_damping_enable:
            velocity_status = 'VELOCITY_DAMPING_DISABLED'

        z_meas, z_source, z_fresh = self._resolve_staged_pid_vertical_measurement()
        self._staged_pid_last_z_fresh = z_fresh
        self._staged_pid_last_z_meas = z_meas
        self._staged_pid_last_z_source = z_source
        self._staged_pid_last_allow_z = False

        if not z_fresh or z_meas is None:
            self._z_pid.reset()
            return vx_body, vy_body, 0.0, yaw, f'HOLD_MAINTAIN_{velocity_status}_NO_DEPTH'

        err_z_m = float(z_meas - self._desired_vertical_dist)
        vz_body = self._hold_maintain_z_kp_scale * self._z_pid.update(err_z_m, dt_ctrl)
        vz_body = clamp(vz_body, self._max_vz)
        source_suffix = str(z_source).upper()
        return vx_body, vy_body, vz_body, yaw, f'HOLD_MAINTAIN_{velocity_status}_DEPTH_{source_suffix}'

    def _target_pose_cb(self, msg) -> None:
        '''Extract the depth information from the latest Pose estimate.'''

        self._latest_target_point = msg
        self._last_target_point_time = self.get_clock().now()

    def _dvl_altitude_cb(self, msg: Dvl) -> None:
        # Phase 2 staged PID fallback. target_point remains primary while fresh.
        self._latest_depth_measurement = float(np.abs(msg.altitude))
        self._last_depth_time = self.get_clock().now()

    def _mavros_altitude_cb(self, msg: Float64) -> None:
        # Compatibility hook. The default staged PID fallback uses DVL.
        self._latest_depth_measurement = float(np.abs(msg.data))
        self._last_depth_time = self.get_clock().now()

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        '''
        Extract CameraInfo paramters from the CameraInfo topic.
            [fx  0 cx]
        K = [ 0 fy cy]
            [ 0  0  1]
        '''
        _camera_info_msg = msg
        k = _camera_info_msg.k  # extract the intrinsic camera matrix
        self.fx = k[0]
        self.fy = k[4]
        self.cx = 320 #k[2]
        self.cy = 320 #k[5]
    
    def _vel_measurement_cb(self, msg: TwistStamped) -> None:
        t = msg.twist
        current_vel = np.array([
            t.linear.x,  t.linear.y,  t.linear.z,
            t.angular.x, t.angular.y, t.angular.z
        ])
        self.latest_velocity_measurement = current_vel
        self._last_velocity_measurement_time = self.get_clock().now()

    def _target_observation_cb(self, msg: TargetObservation) -> None:
        pass

    def _detection_cb(self, detection_2D_msg: Detection2D):
        """This callback reads an incoming detection from the object detection model, extracts the detection with the desired class_id
        with the highest confidence, and then extracts it"s bounding box.
        
        Args:
        detection_array_msg (Detection2DArray) : Array with detection messages

        Returns:
        determines which control method to use, and computes corresponding errors for the PID control signals    

        """
        
        # self.get_logger().info(
        #     f'\nReceived a position of x={position.x:6.2f}, y={position.y:6.2f}\n'
        #     f'position setpoint at x={self.setpoint_x:6.2f}, y={self.setpoint_y:6.2f}',
        #     throttle_duration_sec=1,
        # )

        if self._use_target_observation and self._controller_mode == 'staged_pid':
            # Compatibility path only: the tracker/selector TargetObservation
            # contract is the primary staged-PID gate when enabled.
            return

        self.get_logger().info(f'Detection Recieved!', throttle_duration_sec=2.0)

        now = self.get_clock().now()
        self._last_obs_time = now
        self._latest_detection_time = now
        self._latest_obs = detection_2D_msg.bbox
        best_bbox = self._latest_obs
        
        half_w = best_bbox.size_x / 2.0
        half_h = best_bbox.size_y / 2.0

        self.latest_centroid = [best_bbox.center.position.x ,best_bbox.center.position.y]

        centroid_err_x = self.latest_centroid[0] - self._staged_pid_target_centroid[0]
        centroid_err_y = self.latest_centroid[1] - self._staged_pid_target_centroid[1]

        err_summary = Vector3()
        err_summary.x = float(centroid_err_x)
        err_summary.y = float(centroid_err_y)
        err_summary.z = float(self._latest_depth_measurement - self._desired_target_dist)
        self._centroid_error_publisher.publish(err_summary)

        self._update_target_tracking_from_detection(now)

    def _tick(self) -> None:
        print("tick called")
        now = self.get_clock().now()
        dt_obs = (now - self._last_obs_time).nanoseconds * 1e-9
        dt_ctrl = max(1e-4, (now - self._last_tick_time).nanoseconds * 1e-9)
        self._last_tick_time = now

        self._tick_staged_pid(dt_ctrl, dt_obs)

    def _tick_staged_pid(self, dt_ctrl: float, dt_obs: float) -> None:
        obs_fresh = self._latest_obs is not None and dt_obs <= self._timeout_sec
        if self._use_target_observation:
            print("`_use_target_observation` on in _tick_staged_pid()")
            obs_fresh = (
                self._latest_target_observation is not None and
                dt_obs <= self._timeout_sec
            )
            if should_enter_hold_maintain(
                use_target_observation=self._use_target_observation,
                observation_received=self._latest_target_observation is not None,
                observation_fresh=obs_fresh,
                selected_valid=self._target_observation_valid,
            ):
                print("originally `_set_target_tracked(False)` but turned off now")
                self._set_target_tracked(False)
                self._clear_staged_pid_completion()
                self._publish_vs_control_finish(False)
                self.set_state(self.ControllerState.BREAK)
                state = self._target_observation_state if self._latest_target_observation is not None else 'NO_OBSERVATION'
                reason = self._target_observation_reason if self._latest_target_observation is not None else 'not_received'
                vx_body, vy_body, vz_body, yaw, maintain_status = self._compute_hold_maintain_command(dt_ctrl)
                self.v_last_valid = np.array([vx_body, vy_body, vz_body, 0.0, 0.0, yaw])
                self._publish_status(f'HOLD_SELECTED_INVALID_{state}_{reason}_{maintain_status}')
                self._publish_is_locked('False')
                self._publish_cmd_direct_body_with_slew(vx_body, vy_body, vz_body, yaw, dt_ctrl)
                return
        #else:
            #print("`_use_target_observation` off in _tick_staged_pid()")
        self._update_target_tracked_from_tick()
        self._publish_is_locked(str(self._target_tracked))

        if not self._target_tracked:
            self.set_state(self.ControllerState.BREAK)
            self.v_last_valid = np.zeros(6)
            self._clear_staged_pid_completion()
            self._publish_vs_control_finish(False)
            self._publish_status('TRACKING_STAGED_PID_NO_TARGET')
            self._publish_cmd_direct_body_with_slew(0.0, 0.0, 0.0, 0.0, dt_ctrl)
            return

        if self._staged_pid_completion_latch and self._vs_control_finished:
            # Handoff to the external operation state machine. Continue publishing
            # zero velocity so MAVROS does not time out the setpoint while complete.
            self.set_state(self.ControllerState.COMPLETE)
            self.v_last_valid = np.zeros(6)
            self._publish_vs_control_finish(True)
            self._publish_status('COMPLETE_STAGED_PID_LATCHED')
            self._publish_cmd_direct_body_with_slew(0.0, 0.0, 0.0, 0.0, dt_ctrl)
            return

        if obs_fresh and hasattr(self, 'latest_centroid'):
            vx_body, vy_body, vz_body, yaw, status, _allow_z, z_meas, z_fresh, z_source = self._compute_staged_pid_command(
                obs_fresh,
                dt_ctrl,
            )
            completion_reached = self._update_staged_pid_completion_gate(
                obs_fresh=obs_fresh,
                z_meas=z_meas,
                z_fresh=z_fresh,
            )
            if completion_reached:
                # Handoff to the external operation state machine. Continue publishing
                # zero velocity so MAVROS does not time out the setpoint while complete.
                self.set_state(self.ControllerState.COMPLETE)
                self._vs_control_finished = True
                self.v_last_valid = np.zeros(6)
                self._publish_vs_control_finish(True)
                self._publish_status(f'COMPLETE_STAGED_PID_{z_source.upper()}')
                self._publish_cmd_direct_body_with_slew(0.0, 0.0, 0.0, 0.0, dt_ctrl)
                return
            self.set_state(self.ControllerState.VISUAL)
            self._publish_vs_control_finish(False)
        else:
            #self._staged_pid_state = self.StagedPIDState.HORIZONTAL
            #print(f"`self._staged_pid_state = self.StagedPIDState.HORIZONTAL")
            self._reset_vertical_gate()
            self._clear_staged_pid_completion()
            self._publish_vs_control_finish(False)
            vx_body, vy_body, vz_body, yaw = self._tracked_horizontal_stale_command()
            status = 'TRACKING_STAGED_PID_HORIZONTAL_STALE'

        self.v_last_valid = np.array([vx_body, vy_body, vz_body, 0.0, 0.0, yaw])
        self._publish_status(status)
        self._publish_cmd_direct_body_with_slew(vx_body, vy_body, vz_body, yaw, dt_ctrl)

    def set_state(self, new_state):
        if new_state != self.current_state:
            # Logging the change
            self.get_logger().info(f"State transition: {self.current_state.name} -> {new_state.name}")
            
            # Update the variable
            self.current_state = new_state
            
            # PUBLISH the new state name
            self._publish_status(str(self.current_state.name))
    
    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _publish_vs_control_finish(self, finished: bool) -> None:
        msg = Bool()
        msg.data = bool(finished)
        self._vs_control_finish_pub.publish(msg)

    def _publish_is_locked(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._target_locked_pub.publish(msg)

    def _publish_cmd_direct_body(self, vx: float, vy: float, vz: float, yaw: float) -> None:
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

        sc2_cmd_linear = Vector3()
        sc2_cmd_linear.x = vx
        sc2_cmd_linear.y = vy
        sc2_cmd_linear.z = vz

        sc2_cmd_angular = Vector3()
        sc2_cmd_angular.x = 0.0
        sc2_cmd_angular.y = 0.0
        sc2_cmd_angular.z = yaw

        cmd_vec_6 = Vector6()
        cmd_vec_6.linear = sc2_cmd_linear
        cmd_vec_6.angular = sc2_cmd_angular
        self._sc2_cmd_pub_vec_6.publish(cmd_vec_6)

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

    def _publish_cmd_direct_body_with_slew(self, vx: float, vy: float, vz: float, yaw: float, dt: float) -> None:
        pvx, pvy, pvz, pyaw = self._last_cmd
        svx = self._slew_axis(vx, pvx, self._slew_vx, dt)
        svy = self._slew_axis(vy, pvy, self._slew_vy, dt)
        svz = self._slew_axis(vz, pvz, self._slew_vz, dt)
        syaw = self._slew_axis(yaw, pyaw, self._slew_yaw, dt)
        self._publish_cmd_direct_body(svx, svy, svz, syaw)

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
